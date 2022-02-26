/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.component;

import frc.util.Component;

//Implement component so that this can be included in the main loop
public class ExampleComponent implements Component{

    //Store a static instance and create it for the singleton pattern
    private static ExampleComponent instance = new ExampleComponent();
    double mSpeed;

    private ExampleComponent() {
        //Just here to remove the public constructor
    }

    //Used to set the speed outside of this file
    public void setSpeed(double speed) {
        mSpeed = speed;
    }

    @Override
    public void execute() {
        //Code ran every loop
    }

    public static ExampleComponent getInstance() {
        return instance;
    }
}
