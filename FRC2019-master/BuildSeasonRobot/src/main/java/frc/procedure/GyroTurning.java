/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.procedure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.component.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.sensor.PIDControl;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.kauailabs.navx.frc.AHRS;


/**
 * Add your docs here.
 */
public class GyroTurning {
    AHRS gyro;
    PIDControl turnController;
    double turnRate;
    Drive drive = Drive.getInstance();

    private static GyroTurning instance;
    
    private GyroTurning() {
        init();
    }
    
	public void init() {
        turnController = new PIDControl(0.02, 0, 0);

        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setTolerance(1);
        turnController.setContinuous();
        turnController.setSetpoint(0);

        try {
            gyro = new AHRS(SPI.Port.kMXP);
            gyro.setName("Gyro", "Angle");
            gyro.reset();
        } catch(Exception e) {System.out.println("Gyro could not initialize!");}

        turnController.setName("GyroTuning");
        turnController.initSmartDashboard("GyroTuning");
    }

    public double run() {
        turnController.updateFromSmartDashboard("GyroTuning");
        
        double motorSpeed = turnController.calculate(gyro.getAngle());
        drive.driveCartesian(drive.getHorizontalSpeed(), drive.getVerticalSpeed(), motorSpeed);
        return turnRate;
    }

    public double getSpeed() {
        turnController.updateFromSmartDashboard("GyroTuning");
        double motorSpeed = turnController.calculate(gyro.getAngle());
        System.out.println("Gyro Angle: " + gyro.getAngle() + " Motor Speed: " + motorSpeed + " Setpoint: " + turnController.getSetpoint());
        return motorSpeed;
    }

    //Scale the angle input between -180 and 180
    public void setAngle(double angle) {
        turnController.setSetpoint(angle);
    }

    public double getAngle() {
        //Returns the remainder where the quotient = the closest integer (Ex: -181/380 = -1r179 returns 179)
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public boolean onTarget() {
        return turnController.onTarget();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetPID() {
        turnController.reset();
    }

    public static GyroTurning getInstance() {
        if(instance == null) {
            instance = new GyroTurning();
        }

        return instance;
    }
}
