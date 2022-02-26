package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.component.Arm;
import frc.component.Drive;
import frc.component.Intake;
import frc.component.Jacks;
import frc.component.Wrist;
import frc.procedure.CameraAlign;
import frc.procedure.GyroTurning;
import frc.sensor.Limelight;
import frc.util.Component;
import frc.util.Debug;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

/**
 */

public class Robot extends TimedRobot {
    ArrayList<Component> components = new ArrayList<>();

    OI.DriverProfile controlProfile;
    XboxController driver;

    Drive drive;
    Jacks jacks;
    Intake intake;
    Wrist wrist;
    Arm arm;

    private String m_driverSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        CameraServer.getInstance().startAutomaticCapture();

        // Init here, should be overwritten in telopinit
        controlProfile = OI.getProfile(OI.MAIN_DRIVER_PROFILE);
        driver = controlProfile.driver;

        // Store in cleaner variables
        drive = Drive.getInstance();
        jacks = Jacks.getInstance();
        intake = Intake.getInstance();
        wrist = Wrist.getInstance();
        arm = Arm.getInstance();

        GyroTurning.getInstance().resetGyro();

        components.add(drive);
        components.add(jacks);
        components.add(intake);
        components.add(wrist);
        components.add(arm);

        Debug.init();

        drive.invertX(true);

        // Put drive profiles on smartDashboard
        m_chooser.setDefaultOption("Feaven", OI.MAIN_DRIVER_PROFILE);
        m_chooser.addOption("Logitech", OI.LOGITECH_CONTROLLER);
        m_chooser.addOption("Admin", OI.ADMIN_PROFILE);
        m_chooser.addOption("Joystick", OI.JOYSTICK_CONTROLLER);

        SmartDashboard.putData("Driver Mode", m_chooser);

        Limelight.getInstance().setLightState(Limelight.LightMode.OFF);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        if (m_driverSelected != m_chooser.getSelected()) {
            m_driverSelected = m_chooser.getSelected();
            controlProfile = OI.getProfile(m_driverSelected);
        }
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        // Set the camera values
        Limelight.getInstance().setPipeline(0);
        // CameraAlign camera = CameraAlign.getInstance();
        // camera.align(5);
        // while(!camera.isCompleted()) {
        //     camera.run();
        // }
        GyroTurning.getInstance().resetGyro();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {
        // Only swap profile if the profile was changed since last teleop
        if (m_driverSelected != m_chooser.getSelected()) {
            m_driverSelected = m_chooser.getSelected();
            controlProfile = OI.getProfile(m_driverSelected);
        }

        // Set the camera values
        Limelight.getInstance().setPipeline(0);
    }

    /**
     * This function is called periodically during operator control.
     */
    // leftEncoder = 8,9
    GyroTurning gyro = GyroTurning.getInstance();
    @Override
    public void teleopPeriodic() {
        // double[] yCorners = Limelight.getInstance().getYCorners();

        // double leftCorner = yCorners[1];
        // double rightCorner = yCorners[0];

        // System.out.println("LeftCorner: " + leftCorner + " RightCorner: " + rightCorner + " Bool: " + (leftCorner > rightCorner));

        controlProfile.drive();
        controlProfile.cameraDrive();
        if(driver.getBButton()) {
            gyro.resetGyro();
            gyro.resetPID();
        }
        // if(driver.getAButton()) {
        //     gyro.resetGyro();
        //     gyro.resetPID();
        // }

        // if(driver.getBButton()) {
        //     gyro.setAngle(90);
        //     drive.driveCartesian(0, 0, gyro.getSpeed());
        // }

        // if(driver.getXButton()) {
        //     gyro.setAngle(-90);
        //     drive.driveCartesian(0, 0, gyro.getSpeed());
        // }

        // if(driver.getYButton()) {
        //     gyro.setAngle(-180);
        //     drive.driveCartesian(0, 0, gyro.getSpeed());
        // }
        
        controlProfile.controlArm();
        controlProfile.controlArmPID();

        controlProfile.controlFrontJacks();
        controlProfile.controlRearJackWheel();
        controlProfile.controlRearJacks();

        controlProfile.controlWrist();
        controlProfile.controlIntake();

        updateAllComponents();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }

    @Override
    public void disabledInit() {
        Limelight.getInstance().setLightState(Limelight.LightMode.OFF);
    }

    public void updateAllComponents() {
        // run all of our components
        for (Component component : components) {
            component.execute();
        }
    }
}
