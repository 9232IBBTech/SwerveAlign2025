/*package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;

public class AlignToBestTargetCommand extends Command {

    private final SwerveSubsystem drive;
    private final double lateralOffsetMeters;

    private final Timer atSetpointTimer = new Timer();
    private boolean isTimerStarted = false;
    private static final double VERIFICATION_DURATION_SECONDS = 0.5;
    
    private int wantedID = -1;
    private boolean isTargetLost = false;

    private static final double TARGET_DISTANCE_METERS = 0.5;
    private static final double DISTANCE_TOLERANCE = 0.02;
    private static final double LATERAL_TOLERANCE = 0.02;
    private static final double YAW_TOLERANCE_RAD = Math.toRadians(1);

    private final PIDController xController = new PIDController(2.4, 0, 0);
    private final PIDController yController = new PIDController(2.4, 0, 0);
    private final PIDController thetaController = new PIDController(3.5, 0, 0);

    public AlignToBestTargetCommand(SwerveSubsystem drive, double lateralOffsetMeters) {
        this.drive = drive;
        this.lateralOffsetMeters = lateralOffsetMeters;

        xController.setTolerance(DISTANCE_TOLERANCE);
        yController.setTolerance(LATERAL_TOLERANCE);
        thetaController.setTolerance(YAW_TOLERANCE_RAD);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();

        Optional<SightedTarget> bestSightingOpt = vision.getBestSightedTarget();

        if (bestSightingOpt.isPresent()) {
            this.wantedID = bestSightingOpt.get().target.getFiducialId();
            this.isTargetLost = false;
            System.out.println("AlignToBestTargetCommand başladı! Kilitlenilen ID: " + this.wantedID);
        } else {
            System.out.println("AlignToBestTargetCommand: Başlangıçta hedef bulunamadı, komut bitiriliyor.");
            this.isTargetLost = true;
        }
    }

    @Override
    public void execute() {
        if (isTargetLost) {
            drive.drive(new ChassisSpeeds()); // Güvenlik için durduğundan emin ol
            return;
        }

        Optional<SightedTarget> sightedTargetOpt = vision.findBestSightingForId(wantedID);

        if (sightedTargetOpt.isPresent()) {
            
            Transform3d robotToTarget = vision.getRobotToWantedTarget(sightedTargetOpt).get();
            Translation3d translation = robotToTarget.getTranslation();

            double xSpeed = !xController.atSetpoint() ? xController.calculate(translation.getX(), TARGET_DISTANCE_METERS) : 0;
            double ySpeed = !yController.atSetpoint() ? yController.calculate(translation.getY(), -lateralOffsetMeters) : 0;
            double rotationalSpeed = !thetaController.atSetpoint() ? thetaController.calculate(robotToTarget.getRotation().getZ(), Math.PI) : 0;

            drive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -rotationalSpeed));
            
            updateSmartDashboard(true, translation, robotToTarget.getRotation().getZ(), xSpeed, ySpeed, rotationalSpeed);

        } else {
            drive.drive(new ChassisSpeeds());
            this.isTargetLost = true;
            
            updateSmartDashboard(false, null, 0, 0, 0, 0);
            System.out.println(wantedID + " ID'li hedef kaybedildi.");
        }
    }
    
    @Override
    public boolean isFinished() {
        if (isTargetLost) {
            atSetpointTimer.stop();
            atSetpointTimer.reset();
            isTimerStarted = false;
            return true; // Hedef kaybolduysa bekleme yapmadan bitir.
        }
    
        // O anki hedefi görerek tolerans içinde olup olmadığımızı kontrol edelim.
        boolean isCurrentlyAtSetpoint = false;
        Optional<SightedTarget> currentSightingOpt = vision.findBestSightingForId(wantedID);
    
        if (currentSightingOpt.isPresent()) {
            Transform3d robotToTarget = vision.getRobotToWantedTarget(currentSightingOpt).get();
            Translation3d translation = robotToTarget.getTranslation();
    
            // Manuel olarak tolerans kontrolü
            boolean isXCorrect = Math.abs(translation.getX() - TARGET_DISTANCE_METERS) <= DISTANCE_TOLERANCE;
            boolean isYCorrect = Math.abs(translation.getY() - (-lateralOffsetMeters)) <= LATERAL_TOLERANCE;
            
            // PID'nin setpoint'i Math.PI, mevcut ölçümümüz ise ...getZ(). İkisinin arasındaki hatayı kontrol ediyoruz.
            thetaController.setSetpoint(Math.PI);
            boolean isThetaCorrect = Math.abs(thetaController.getPositionError()) <= YAW_TOLERANCE_RAD;
            
            // Genel durum: Hem PID'ler hem de manuel kontrol tolerans içinde mi?
            isCurrentlyAtSetpoint = isXCorrect && isYCorrect && isThetaCorrect;
        }
    
        // KARARLILIK KONTROL MANTIĞI
        if (isCurrentlyAtSetpoint) {
            // Eğer şu an hedefteysek...
            if (!isTimerStarted) {
                // ...ve zamanlayıcı daha önce başlamadıysa, şimdi başlat.
                atSetpointTimer.start();
                isTimerStarted = true;
            }
            // Zamanlayıcının istenen süreyi (0.2sn) doldurup doldurmadığını kontrol et.
            // Eğer süre dolduysa, komut gerçekten bitmiştir.
            return atSetpointTimer.hasElapsed(VERIFICATION_DURATION_SECONDS);
        } else {
            // Eğer şu an hedefte değilsek...
            // ...zamanlayıcıyı durdur ve sıfırla.
            atSetpointTimer.stop();
            atSetpointTimer.reset();
            isTimerStarted = false;
            // Komut bitmedi, devam etmeli.
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Hedef kaybolmadıysa ve komut kesintiye uğramadıysa,
        // son bir kontrol yaparak gerçekten hedefte miyiz diye bak.
        if (!isTargetLost && !interrupted) {
            Optional<SightedTarget> finalCheckOpt = vision.findBestSightingForId(wantedID);
            if (finalCheckOpt.isPresent()) {
                Transform3d robotToTarget = vision.getRobotToWantedTarget(finalCheckOpt).get();
                Translation3d translation = robotToTarget.getTranslation();
                double finalYawError = Math.abs(robotToTarget.getRotation().getZ() - Math.PI);

                boolean isTrulyFinished = 
                    Math.abs(translation.getX() - TARGET_DISTANCE_METERS) <= DISTANCE_TOLERANCE &&
                    Math.abs(translation.getY() - (-lateralOffsetMeters)) <= LATERAL_TOLERANCE &&
                    finalYawError <= YAW_TOLERANCE_RAD;
                
                if (isTrulyFinished) {
                    System.out.println("AlignToBestTargetCommand başarıyla tamamlandı.");
                } else {
                    System.out.println("AlignToBestTargetCommand bitti, ancak son kontrol tolerans dışında kaldı.");
                }
            } else {
                System.out.println("AlignToBestTargetCommand: Son kontrol sırasında hedef kaybedildi.");
            }
        }
        
        drive.drive(new ChassisSpeeds());
        System.out.println("AlignToBestTargetCommand sonlandı. (Kesinti: " + interrupted + ")");
    }
    
    private void updateSmartDashboard(boolean isVisible, Translation3d trans, double yawRad, double xSpeed, double ySpeed, double rotSpeed) {
        SmartDashboard.putBoolean("Align/Target Visible", isVisible);
        
        if (isVisible) {
            // --- X Ekseni (İleri/Geri) ---
            SmartDashboard.putNumber("AlignDebug/X/Measurement", trans.getX());
            SmartDashboard.putNumber("AlignDebug/X/Setpoint", TARGET_DISTANCE_METERS);
            SmartDashboard.putNumber("AlignDebug/X/Error", xController.getPositionError());
            SmartDashboard.putBoolean("AlignDebug/X/atSetpoint", xController.atSetpoint());
            SmartDashboard.putNumber("AlignDebug/X/OutputSpeed_Final", -xSpeed);

            // --- Y Ekseni (Sağ/Sol) ---
            SmartDashboard.putNumber("AlignDebug/Y/Measurement", trans.getY());
            SmartDashboard.putNumber("AlignDebug/Y/Setpoint", -lateralOffsetMeters);
            SmartDashboard.putNumber("AlignDebug/Y/Error", yController.getPositionError());
            SmartDashboard.putBoolean("AlignDebug/Y/atSetpoint", yController.atSetpoint());
            SmartDashboard.putNumber("AlignDebug/Y/OutputSpeed", ySpeed);

            // --- Theta Ekseni (Dönüş) ---
            SmartDashboard.putNumber("AlignDebug/Theta/Measurement (Deg)", Math.toDegrees(yawRad));
            SmartDashboard.putNumber("AlignDebug/Theta/Setpoint", 0);
            SmartDashboard.putNumber("AlignDebug/Theta/Error (Deg)", Math.toDegrees(thetaController.getPositionError()));
            SmartDashboard.putBoolean("AlignDebug/Theta/atSetpoint", thetaController.atSetpoint());
            SmartDashboard.putNumber("AlignDebug/Theta/OutputSpeed_Final", -rotSpeed);
        } else {
            SmartDashboard.putNumber("AlignDebug/X/Error", 0);
            SmartDashboard.putBoolean("AlignDebug/X/atSetpoint", false);
            SmartDashboard.putNumber("AlignDebug/X/OutputSpeed_Final", 0);

            SmartDashboard.putNumber("AlignDebug/Y/Error", 0);
            SmartDashboard.putBoolean("AlignDebug/Y/atSetpoint", false);
            SmartDashboard.putNumber("AlignDebug/Y/OutputSpeed", 0);

            SmartDashboard.putNumber("AlignDebug/Theta/Error (Deg)", 0);
            SmartDashboard.putBoolean("AlignDebug/Theta/atSetpoint", false);
            SmartDashboard.putNumber("AlignDebug/Theta/OutputSpeed_Final", 0);
        }
    }
}*/