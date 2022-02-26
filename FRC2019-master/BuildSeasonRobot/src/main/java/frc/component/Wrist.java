/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import frc.sensor.PIDControl;
import frc.util.Component;
import frc.util.Debug;

//Implement component so that this can be included in the main loop
public class Wrist implements Component {

    // Store a static instance and create it for the singleton pattern
    private static Wrist instance;
    
    WPI_VictorSPX motor = new WPI_VictorSPX(RobotMap.Motors.wrist);

    PIDControl pid = new PIDControl(0.1);
    boolean closedLoop = false;

    DigitalInput lowerLimitSwitch = new DigitalInput(RobotMap.limitSwitches.wristDown);
    DigitalInput upperLimitSwitch = new DigitalInput(RobotMap.limitSwitches.wristUp);

    Counter wristCounter = new Counter(RobotMap.Encoders.wrist);
    int wristPosition = 0;
    int lastWrist = 0;
    //False = down, true = up
    boolean lastDirectionUp = false;

    double mSpeed;

    private Wrist() {
        // Just here to remove the public constructor
        pid.setInputRange(0, 100);
        pid.setOutputRange(-0.6, 0.6);
        pid.setTolerance(2);
        pid.setSetpoint(15);
    }

    public void setPositionUp() {
        closedLoop = true;
    }

    public void disablePID() {
        closedLoop = false;
    }

    public void setSpeed(double speed) {
        mSpeed = speed;

        if (upperLimitSwitch.get()) {
            wristCounter.reset();
            wristPosition = 0;
            mSpeed = Math.min(mSpeed, 0);
        } else if (lowerLimitSwitch.get()) {
            mSpeed = Math.max(mSpeed, 0);
            wristPosition = 51;
        }
    }

    @Override
    public void execute() {
        if(closedLoop) {
            mSpeed = -pid.calculate(wristPosition);
        }

        int wristCount = wristCounter.get();
        int deltaCounter = wristCount - lastWrist;
        lastWrist = wristCount;
        //Add some directional control to the counter based on the motor
        if(mSpeed > 0) {
            wristPosition -= deltaCounter;
            lastDirectionUp = true;
        }
        else if(mSpeed < 0){
            wristPosition += deltaCounter;
            lastDirectionUp = false;
        }
        //For when the motor is passively falling due to gravity or something, keep going in previous direction
        else {
            wristPosition += (lastDirectionUp) ? -deltaCounter : deltaCounter;
        }

        motor.set(mSpeed);
        System.out.println("WristPosition: " + wristPosition + " Counter Position: " + wristCounter.get());
    }

    public void initDebug() {
        ShuffleboardTab tab = Debug.wrist;

        motor.setName("Motors", "Wrist");

        upperLimitSwitch.setName("Limit Switches", "Upper Limit Wrist");
        lowerLimitSwitch.setName("Limit Switches", "Lower Limit Wrist");

        tab.add(motor);

        tab.add(upperLimitSwitch);
            //.withWidget(BuiltInWidgets.kBooleanBox);
        tab.add(lowerLimitSwitch);

        Debug.limitSwitches.add(upperLimitSwitch);
        Debug.limitSwitches.add(lowerLimitSwitch);

        Debug.motors.add(motor);
    }

    public static Wrist getInstance() {
        if(instance == null) {
            instance = new Wrist();
        }
        return instance;
    }
}
