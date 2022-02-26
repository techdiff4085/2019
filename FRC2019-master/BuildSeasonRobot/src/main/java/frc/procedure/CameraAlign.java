/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.procedure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.component.Drive;
import frc.sensor.Limelight;
import frc.sensor.PIDControl;

/**
 * Add your docs here.
 */
public class CameraAlign {

    private static CameraAlign instance;

    private Limelight camera = Limelight.getInstance();
    private Drive drive = Drive.getInstance();

    private int timeout;
    Timer timer = new Timer();

    public PIDControl strafingController = new PIDControl(0.1, 0.01);
    public PIDControl distanceController = new PIDControl(0.15);
    GyroTurning rotationController = GyroTurning.getInstance();

    double[] hatchAngles = {-90, 90, 0, 151.25, -151.25, 28.75, -28.75};
    double[] hatchRetrieval = {180, -180};
    double[] rocketCargo = {90, -90};

    double[] currentAngles;

    private CameraAlign() {
        strafingController.setOutputRange(-0.5, 0.5);
        distanceController.setOutputRange(-0.5, 0.5);

        strafingController.setInputRange(-27, 27);
        distanceController.setInputRange(0, 100);

        strafingController.setTolerance(0.5);
        distanceController.setTolerance(0.2);

        strafingController.setSetpoint(0);
        distanceController.setSetpoint(4);


        strafingController.setMaxIContribution(0.3f);
        strafingController.setIKickInRate(2);

        // strafingController.initSmartDashboard("CameraStrafe");
        // LiveWindow.add(strafingController);
    }

    public void setAlignHatch() {
        currentAngles = hatchAngles;
        camera.setPipeline(0);
        distanceController.setSetpoint(3.5);
        //Distance Area 3.5
    }

    public void setAlignRetrieval() {
        currentAngles = hatchRetrieval;
        camera.setPipeline(1);
        distanceController.setSetpoint(6.2499);
        //Distance area 6.2499
    }

    public void setAlignRocketCargo() {
        currentAngles = rocketCargo;
        camera.setPipeline(2);
        distanceController.setSetpoint(2.147);
        //Distance area 2.147
    }

    public void run() {
        // strafingController.updateFromSmartDashboard("CameraStrafe");
        if (camera.hasValidTarget()) {
            double angle = rotationController.getAngle(); //adds the angle from center of camera to angle
            double strafingSpeed = -strafingController.calculate(camera.getXAngle());
            double distanceSpeed = -distanceController.calculate(camera.getArea());
            double closestAngle = findClosestDouble(angle, currentAngles);
            rotationController.setAngle(closestAngle);

            // Strafe to the indicated position
            /*   ^
                 0
            -90     90
             -180/180
            ____________
            */
            //-90, 90, 180, -180, 0, 151.25, -151.25, 28.75, -28.75
            drive.driveCartesian(strafingSpeed, distanceSpeed, rotationController.getSpeed());
        }
    }

    // Runs the alignment for a set period of time until the timeout happens or is
    // alligned
    public void align(int timeout) {
        this.timeout = timeout;
        timer.reset();
        timer.start();

        run();
    }

    // Returns if correctly aligned
    public boolean isAlligned() {
        return strafingController.onTarget() && rotationController.onTarget();
    }

    public boolean isCompleted() {
        if (timer.hasPeriodPassed(timeout) || isAlligned()) {
            timer.stop();
            return true;
        }

        return false;
    }

    // Should be called at the start of a move using this
    // So that the derivative error and integral error
    // start correctly
    public void resetPID() {
        strafingController.reset();
        rotationController.resetPID();
    }

    public static CameraAlign getInstance() {
        if (instance == null) {
            instance = new CameraAlign();
        }

        return instance;
    }

    public double findClosestDouble(double n, double[] values) {
        double bestN = values[0];
        double smallestDistance = Math.abs(n - bestN);

        for(int i = 0; i < values.length; i++) {
            if(Math.abs(n - values[i]) < smallestDistance) {
                bestN = values[i];
                smallestDistance = Math.abs(n - bestN);
            }
        }

        return bestN;
    }
}
