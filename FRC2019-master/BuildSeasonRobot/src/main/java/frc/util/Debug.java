/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.component.Arm;
import frc.component.Drive;
import frc.component.Intake;
import frc.component.Jacks;
import frc.component.Wrist;

/**
 * Add your docs here.
 */
public class Debug {

    public static final ShuffleboardTab arm = Shuffleboard.getTab("Arm");
    public static final ShuffleboardTab intake = Shuffleboard.getTab("Intake");
    public static final ShuffleboardTab wrist = Shuffleboard.getTab("Wrist");
    public static final ShuffleboardTab jacks = Shuffleboard.getTab("Jacks");
    public static final ShuffleboardTab drive = Shuffleboard.getTab("Drive Train");
    public static final ShuffleboardTab sensors = Shuffleboard.getTab("Sensors");
    public static final ShuffleboardLayout encoders = sensors.getLayout("Encoders", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(0, 0);

    public static final ShuffleboardLayout limitSwitches = sensors.getLayout("Limit Switches", BuiltInLayouts.kList)
        .withSize(2, 8)
        .withPosition(3, 0);

    public static final ShuffleboardLayout motors = sensors.getLayout("Motors", BuiltInLayouts.kList)
        .withSize(3, 10)
        .withPosition(6, 0);

    //Initialize the shuffleboard
    public static void init() {
        Drive.getInstance().initDebug();
        Arm.getInstance().initDebug();
        Wrist.getInstance().initDebug();
        Jacks.getInstance().initDebug();
        Intake.getInstance().initDebug();
    }
}
