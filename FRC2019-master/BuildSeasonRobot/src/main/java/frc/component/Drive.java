/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import frc.util.Component;
import frc.util.Debug;

/**
 * Used for driving Inlcudes both singular side tank drive as well as mecanum
 */
public class Drive implements Component {
    private static Drive instance;

    private WPI_VictorSPX FL;
    private WPI_VictorSPX BL;
    private WPI_VictorSPX FR;
    private WPI_VictorSPX BR;

    private MecanumDrive mecanumDrive;
    // private DifferentialDrive tankDrive;

    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;

    private Encoder leftEncoder = new Encoder(12, 13, false, EncodingType.k4X);
    private Encoder rightEncoder = new Encoder(10, 11, false, EncodingType.k4X);

    private boolean mecanum = false;
    private boolean invertX = false;

    private double ySpeed, xSpeed, rotate;

    private Drive() {
        FL = new WPI_VictorSPX(RobotMap.Motors.FLDrive);
        BL = new WPI_VictorSPX(RobotMap.Motors.BLDrive);
        FR = new WPI_VictorSPX(RobotMap.Motors.FRDrive);
        BR = new WPI_VictorSPX(RobotMap.Motors.BRDrive);

        leftMotors = new SpeedControllerGroup(FL, BL);
        rightMotors = new SpeedControllerGroup(FR, BR);
        rightMotors.setInverted(true);

        mecanumDrive = new MecanumDrive(FL, BL, FR, BR);
        // tankDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    // Mecanum Drive X is forwards/backwards y is left/right
    public void driveCartesian(double ySpeed, double xSpeed, double rotate) {
        mecanum = true;
        this.ySpeed = ySpeed;
        this.xSpeed = (invertX) ? -xSpeed : xSpeed;
        this.rotate = rotate;
    }

    // Tank Drive
    public void tankDrive(double leftSpeed, double rightSpeed) {
        mecanum = false;
        xSpeed = leftSpeed;
        ySpeed = rightSpeed;
    }

    // Ran at the end of teleopPeriodic and autonomousPeriodic
    @Override
    public void execute() {
        if (mecanum) {
            mecanumDrive.driveCartesian(ySpeed, xSpeed, rotate);
        } else {
            // X = left Y = right
            // tankDrive.tankDrive(xSpeed, ySpeed);
        }
    }

    public double getHorizontalSpeed() {
        return ySpeed;
    }

    public double getVerticalSpeed() {
        return xSpeed;
    }

    public double getRotationalSpeed() {
        return rotate;
    }

    public void invertX(boolean b) {
        invertX = b;
    }

    public void initDebug() {
        ShuffleboardTab tab = Debug.drive;
        FL.setName("Motors", "Front Left");
        BL.setName("Motors", "Back Left");
        FR.setName("Motors", "Front Right");
        BR.setName("Motors", "Back Right");

        mecanumDrive.setName("Drive System", "Mecanum");

        leftEncoder.setName("Encoder", "Left");
        rightEncoder.setName("Encoder", "Right");

        tab.add(FL)
            .withPosition(0, 0);
        tab.add(FR)
            .withPosition(2, 0);
        tab.add(BL)
            .withPosition(0, 1);
        tab.add(BR)
            .withPosition(2, 1);

        tab.add(mecanumDrive)
            .withPosition(0, 2);

        tab.add(leftEncoder)
            .withPosition(5, 0);
        tab.add(rightEncoder)
            .withPosition(5, 1);

        Debug.encoders.add(leftEncoder);
        Debug.encoders.add(rightEncoder);

        Debug.motors.add(FL);
        Debug.motors.add(FR);
        Debug.motors.add(BL);
        Debug.motors.add(BR);
    }

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }
}
