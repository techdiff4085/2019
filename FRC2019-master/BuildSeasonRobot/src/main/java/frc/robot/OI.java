package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.component.Arm;
import frc.component.Drive;
import frc.component.Intake;
import frc.component.Jacks;
import frc.component.Wrist;
import frc.procedure.CameraAlign;
import frc.sensor.Limelight;
import frc.sensor.Limelight.LightMode;

public class OI {
    // Administrator, DriveTrials, Competition
    public static final String ADMIN_PROFILE = "Administrator";
    public static final String MAIN_DRIVER_PROFILE = "Feaven";
    public static final String LOGITECH_CONTROLLER = "Logitech";
    public static final String JOYSTICK_CONTROLLER = "Joystick";

    // Create and return the profile
    public static DriverProfile getProfile(String profile) {
        switch (profile) {
        default:
        case MAIN_DRIVER_PROFILE:
            return new DriverProfile();
        case LOGITECH_CONTROLLER:
            return new LogitechProfile();
        case ADMIN_PROFILE:
            return new AdminProfile();
        case JOYSTICK_CONTROLLER:
            return new JoystickProfile();
        }
    }

    static class DriverProfile {
        XboxController driver = new XboxController(0);
        XboxController assistant = new XboxController(1);

        JoystickButton intakeOutButton = new JoystickButton(assistant, 6); // right bumper
        JoystickButton intakeInButton = new JoystickButton(assistant, 5); // left bumper

        JoystickButton frontJackExtendButton = new JoystickButton(driver, 6); // Right bumper
        JoystickButton frontJackRetractButton = new JoystickButton(driver, 5); // Left bumper

        Drive drive = Drive.getInstance();
        Arm arm = Arm.getInstance();
        Jacks jacks = Jacks.getInstance();
        Wrist wrist = Wrist.getInstance();
        Intake intake = Intake.getInstance();

        CameraAlign cameraAlign = CameraAlign.getInstance();
        Limelight camera = Limelight.getInstance();

        public double getHorizontalDriveSpeed() {
            // Deadzone (Controller has a bit of issues centering from the right) (Think
            // this is only the Bluetooth one)
            double controllerValue = driver.getX(Hand.kLeft);
            return (Math.abs(controllerValue) < 0.13f) ? 0 : controllerValue;
        }

        public double getVerticalDriveSpeed() {
            return driver.getY(Hand.kLeft);
        }

        public double getRotationalDriveSpeed() {
            return driver.getX(Hand.kRight);
        }

        public void drive() {
            // Drive based off controls
            if(driver.getPOV() == 90) {
                drive.driveCartesian(0.3, 0, 0);
            }
            else if(driver.getPOV() == 270) {
                drive.driveCartesian(-0.3, 0, 0);
            }
            else {
                drive.driveCartesian(getHorizontalDriveSpeed(), getVerticalDriveSpeed(), getRotationalDriveSpeed());
            }
        }

        public void cameraDrive() {
            // Drive based off camera

            //Hatch Alignment
            if(driver.getAButton()) {
                if(driver.getAButtonPressed()) {
                    camera.setLightState(LightMode.ON);
                    cameraAlign.resetPID();
                    cameraAlign.setAlignHatch();
                    camera.takeSnapshot();
                }
                cameraAlign.run();
            }
            else if(driver.getAButtonReleased()) {
                camera.takeSnapshot();
                camera.setLightState(LightMode.OFF);
            }
            //Retrieval Alignment
            else if(driver.getXButton()) { //Hatch retrieval
                if(driver.getXButtonPressed()) {
                    camera.setLightState(LightMode.ON);
                    cameraAlign.resetPID();
                    cameraAlign.setAlignRetrieval();
                    camera.takeSnapshot();
                }
                cameraAlign.run();
            }
            else if(driver.getXButtonReleased()) {
                camera.takeSnapshot();
                camera.setLightState(LightMode.OFF);
            }
            //Cargo Alignment
            else if(driver.getYButton()) {
                if(driver.getYButtonPressed()) {
                    camera.setLightState(LightMode.ON);
                    cameraAlign.resetPID();
                    cameraAlign.setAlignRocketCargo();
                    camera.takeSnapshot();
                }
                cameraAlign.run();
            }
            else if(driver.getYButtonReleased()) {
                camera.takeSnapshot();
                camera.setLightState(LightMode.OFF);
            }
            
            //Manual Light Control
            if(driver.getRawButtonPressed(8)) {
                camera.setLightState(LightMode.ON);
            }
            else if(driver.getRawButtonPressed(7)) {
                camera.setLightState(LightMode.OFF);
            }
        } 

        public void controlArm() {
            double speed = -assistant.getY(Hand.kLeft) * 0.65f;

            if(assistant.getRawButton(7)){
                arm.overrideLimit();
            }

            // Switch back to user control if using it
            if (Math.abs(speed) > 0.2) {
                arm.disablePID();
            }

            arm.setSpeed(speed);
        }

        public void controlArmPID() {
            // Toggle the PIDControl on the arm

            if (assistant.getAButton()) {
                arm.enablePID();
                arm.setLow();
            } else if (assistant.getXButton()) {
                arm.enablePID();
                arm.setMiddle();
            } else if (assistant.getYButton()) {
                arm.enablePID();
                arm.setHigh();
            } else if (assistant.getBButton()) {
                arm.enablePID();
                arm.setDown();
            }
        }

        public void controlWrist() {
            double speed = -assistant.getY(Hand.kRight) * 0.6f;
            if(assistant.getRawButtonPressed(8)) {
                wrist.setPositionUp();
            }

            if(Math.abs(speed) > 0.1) {
                wrist.disablePID();
            }

            wrist.setSpeed(speed);
        }

        public void controlFrontJacks() {
            if (frontJackExtendButton.get()) {
                jacks.setFrontSpeed(1f);
            } else if (frontJackRetractButton.get()) {
                jacks.setFrontSpeed(-1f);
            } else {
                jacks.setFrontSpeed(0);
            }
        }

        public void controlRearJacks() {
            double speed = assistant.getRawAxis(2) - assistant.getRawAxis(3);
            jacks.setRearSpeed(speed);
        }

        public void controlRearJackWheel() {
            int pov = assistant.getPOV();
            if (pov == 90 || pov == 45 || pov == 135) {
                jacks.setWheelSpeed(-1);
            } else if (pov == 270 || pov == 225 || pov == 315) {
                jacks.setWheelSpeed(1);
            } else {
                jacks.setWheelSpeed(0);
            }
        }

        public void controlIntake() {
            if (intakeOutButton.get()) { // out
                intake.setSpeed(0.5f);
            } else if (intakeInButton.get()) { // in
                intake.setSpeed(-0.5f);
            } else {
                intake.setSpeed(0);
            }
        }
    }

    static class JoystickProfile extends DriverProfile {
        @Override
        public void controlFrontJacks() {
            //Do nothing, inoperable in this mode.
            return;
        }

        @Override
        public void controlArmPID() {
            //Do nothing, inoperable in this mode.
            arm.disablePID();
            return;
        }

        @Override
        public void cameraDrive() {
            //Do nothing, inoperable in this mode.
            return;
        }

        @Override
        public void controlWrist() {
            wrist.setSpeed(assistant.getRawAxis(1));
        }

        @Override
        public void controlRearJacks() {
            //Inoperable, do nothing in this mode
            return;
        }
        @Override
        public void controlArm() {
            //up
            if(assistant.getRawButton(10)) {
                arm.setSpeed(-0.65);
            }
            else if(assistant.getRawButton(11)) {
                arm.setSpeed(0.65);
            }
            else {
                arm.setSpeed(0);
            }
        }

        @Override
        public void controlRearJackWheel() {
            //Do nothing, inoperable in this mode.
            return;
        }

        @Override
        public void controlIntake() {
            if(assistant.getRawButton(7)) {
                intake.setSpeed(-0.5f);
            }
            else if(assistant.getRawButton(6)) {
                intake.setSpeed(0.5f);
            }
            else {
                intake.setSpeed(0);
            }
        }
    }
    // ---------------------------------------Logitech----------------------------------
    static class LogitechProfile extends DriverProfile {
        @Override
        public void controlWrist() {
            wrist.setSpeed(-assistant.getRawAxis(3));
        }

        @Override
        public void controlRearJacks() {
            double jacksSpeed = 0;
            if (assistant.getRawButton(8)) {
                jacksSpeed = -1;
            } else if (assistant.getRawButton(7)) {
                jacksSpeed = 1;
            }

            jacks.setRearSpeed(jacksSpeed);
        }
    }

    // --------------------------------------ADMIN--------------------------------------
    static class AdminProfile extends DriverProfile {

        JoystickButton armUpButton = new JoystickButton(driver, 2); // Xbox B
        JoystickButton armDownButton = new JoystickButton(driver, 1); // xbox A

        JoystickButton frontJackRetractButton = new JoystickButton(driver, 4); // xbox Y
        JoystickButton frontJackExtendButton = new JoystickButton(driver, 3); // xbox X

        JoystickButton intakeOutButton = new JoystickButton(driver, 6); // right bumper
        JoystickButton intakeInButton = new JoystickButton(driver, 5); // left bumper

        @Override
        public double getHorizontalDriveSpeed() {
            // Deadzone (Controller has a bit of issues centering from the right)
            double controllerValue = driver.getX(Hand.kLeft);
            return (Math.abs(controllerValue) < 0.13f) ? 0 : controllerValue;
        }

        @Override
        public void controlWrist() {
            double wristSpeed = driver.getTriggerAxis(Hand.kLeft) - driver.getTriggerAxis(Hand.kRight); // Add the
            wrist.setSpeed(wristSpeed);
        }

        @Override
        public void cameraDrive() {
            if (driver.getRawButtonPressed(7)) {
                cameraAlign.resetPID();
            } else if (driver.getRawButton(7)) {
                cameraAlign.run();
            }
        }

        @Override
        public void controlArm() {
            if (armUpButton.get()) {
                arm.setSpeed(0.4f);
            } else if (armDownButton.get()) {
                arm.setSpeed(-0.4f);
            } else {
                arm.setSpeed(0);
            }
        }

        @Override
        public void controlArmPID() {
            // Toggle the PIDControl on the arm
            if (driver.getRawButtonPressed(8)) {
                // Prevent arm from drastically moving when first pressed
                arm.setSetpoint(arm.getEncoder());
                if (arm.isPIDEnabled()) {
                    arm.disablePID();
                } else {
                    arm.enablePID();
                }
            }

            // PIDControl
            if (arm.isPIDEnabled()) {
                double setPoint = Arm.getInstance().getSetpoint();
                if (driver.getBButtonPressed()) { // Up
                    if (setPoint < 0) {
                        arm.setDown();
                    } else if (setPoint < Arm.ARM_LOW) {
                        arm.setLow();
                    } else if (setPoint < Arm.ARM_MIDDLE) {
                        arm.setMiddle();
                    } else if (setPoint < Arm.ARM_HIGH) {
                        arm.setHigh();
                    }
                } else if (driver.getAButtonPressed()) { // Down
                    if (setPoint > Arm.ARM_HIGH) {
                        arm.setHigh();
                    } else if (setPoint > Arm.ARM_MIDDLE) {
                        arm.setMiddle();
                    } else if (setPoint > Arm.ARM_LOW) {
                        arm.setLow();
                    } else if (setPoint > 0) {
                        arm.setDown();
                    }
                } else if (driver.getStickButtonPressed(Hand.kRight)) {
                    arm.setSetpoint(arm.getSetpoint() - 6);
                }
            }
        }

        @Override
        public void controlFrontJacks() {
            if (frontJackExtendButton.get()) {
                jacks.setFrontSpeed(1);
            } else if (frontJackRetractButton.get()) {
                jacks.setFrontSpeed(-1);
            } else {
                jacks.setFrontSpeed(0);
            }
        }

        @Override
        public void controlRearJacks() {
            int pov = driver.getPOV();
            if (pov == 0 || pov == 45 || pov == 315) {
                jacks.setRearSpeed(1);
            } else if (pov == 180 || pov == 225 || pov == 135) {
                jacks.setRearSpeed(-1);
            } else {
                jacks.setRearSpeed(0);
            }
        }

        @Override
        public void controlIntake() {
            if (intakeOutButton.get()) { // out
                intake.setSpeed(0.5f);
            } else if (intakeInButton.get()) { // in
                intake.setSpeed(-0.5f);
            } else {
                intake.setSpeed(0);
            }
        }
    }
}