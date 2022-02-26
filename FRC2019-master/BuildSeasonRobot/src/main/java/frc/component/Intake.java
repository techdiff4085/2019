/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.component;

import frc.util.Component;
import frc.util.Debug;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;

//Implement component so that this can be included in the main loop
public class Intake implements Component {

    private static Intake instance;
    
    WPI_VictorSPX intakeMotor = new WPI_VictorSPX(RobotMap.Motors.intake);
    double mSpeed;

    private Intake() {
        // Just here to remove the public constructor
    }

    // Store a static instance and create it for the singleton pattern
    public void setSpeed(double Speed) {
        mSpeed = Speed;
    }

    @Override
    public void execute() {
        intakeMotor.set(mSpeed);
        // Code ran every loop
    }

    public void initDebug() {
        ShuffleboardTab tab = Debug.intake;
        intakeMotor.setName("Motors", "Intake");

        tab.add(intakeMotor);

        Debug.motors.add(intakeMotor);
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
