// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsytem.*;
import frc.subsytem.Drive.DriveState;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import org.jetbrains.annotations.NotNull;

import static frc.robot.Constants.IS_PRACTICE;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private double disabledTime = 0;

    private @NotNull Drive drive;
    private @NotNull RobotTracker robotTracker;
    private @NotNull VisionHandler visionHandler;
    private @NotNull Intake intake;
    private @NotNull Arm arm;

    private @NotNull Controller xbox;
    private @NotNull Controller buttonPanel;


    // Autonomous
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final SendableChooser<String> sideChooser = new SendableChooser<>();


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during
     * disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */


    @Override
    public void autonomousInit() {
        drive.configBrake();
        String autoName = autoChooser.getSelected();
        if (autoName != null) {
            AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true);
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }


    /**
     * This method is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        drive.configBrake();
    }


    /**
     * This method is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        AutonomousContainer.getInstance().killAuto();
        disabledTime = Timer.getFPGATimestamp();
    }


    /**
     * This method is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
        if (Timer.getFPGATimestamp() - disabledTime > Constants.COAST_AFTER_DISABLE_TIME) {
            drive.configCoast();
        }
    }


    /**
     * This method is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        drive.setDriveState(DriveState.TELEOP);
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XboxButtons.B)) {
            drive.setAbsoluteZeros();
        }
    }

    public void startSubsystems() {
        drive.start();
        robotTracker.start();
        visionHandler.start();
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), -xbox.getRawAxis(4))
                    .applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), -xbox.getRawAxis(4))
                    .applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs();
        }
    }


    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
    /*
     * Autonomous selection options.
     */
    private static final String kNothingAuto = "do nothing";
    private static final String kConeAuto = "cone";
    private static final String kCubeAuto = "cube";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /*
     * Drive motor controller instances.
     *
     * Change the id's to match your robot.
     * Change kBrushed to kBrushless if you are using NEO's.
     * Use the appropriate other class if you are using different controllers.
     */

    /*
     * Mechanism motor controller instances.
     *
     * Like the drive motors, set the CAN id's to match your robot or use different
     * motor controller classes (TalonFX, TalonSRX, Spark, VictorSP) to match your
     * robot.
     *
     * The arm is a NEO on Everybud.
     * The intake is a NEO 550 on Everybud.
     */

    /*
     * Magic numbers. Use these to adjust settings.
     */


    /**
     * This method is run once when the robot is first started up.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("do nothing", kNothingAuto);
        m_chooser.addOption("cone and mobility", kConeAuto);
        m_chooser.addOption("cube and mobility", kCubeAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        /*
         * Set the arm and intake to brake mode to help hold position.
         * If either one is reversed, change that here too. Arm out is defined
         * as positive, arm in is negative.
         */


        //Our robotInit()


        drive = Drive.getInstance();
        robotTracker = RobotTracker.getInstance();
        visionHandler = VisionHandler.getInstance();
        intake = Intake.getInstance();
        arm = Arm.getInstance();
        xbox = new Controller(0);
        buttonPanel = new Controller(1);

        startSubsystems();
        AutonomousContainer.getInstance().setDebugPrints(true);
        AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        drive::setAutoPath,
                        drive::stopMovement,
                        drive::setAutoRotation,
                        drive::isFinished,
                        drive::getAutoElapsedTime,
                        robotTracker::resetPose,
                        false

                ),
                false,
                null
        );
        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

        sideChooser.setDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        SmartDashboard.putData("Auto choices", autoChooser);
        SmartDashboard.putData("Red or Blue", sideChooser);

        startSubsystems();

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }
    }


    /**
     * This method is called every 20 ms, no matter the mode. It runs after
     * the autonomous and teleop specific period methods.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    }

    /**
     * Used to remember the last game piece picked up to apply some holding power.
     */
    static final int CONE = 1;
    static final int CUBE = 2;
    int lastGamePiece;

    @Override
    public void teleopPeriodic() {
        xbox.update();
        buttonPanel.update();

        // Arm position controls
        if (xbox.getRisingEdge(XboxButtons.X)) {
            // Low position
        } else if (xbox.getRisingEdge(XboxButtons.Y)) {
            // Middle Position
        } else if (xbox.getRisingEdge(XboxButtons.B)) {
            // High Position
        }

        // Intake speed control
        if (xbox.getRawAxis(3) > .1) {
            // Intaking
        } else if (xbox.getRawAxis(2) > .1) {
            // Outtaking
        } else {
            // Holding
        }

        /*
         * Negative signs here because the values from the analog sticks are backwards
         * from what we want. Forward returns a negative when we want it positive.
         */
        drive.swerveDrive(getControllerDriveInputs());

        if (xbox.getRisingEdge(XboxButtons.A)) {
            robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), new Rotation2d()));
        }
    }
}