// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private @NotNull Elevator elevator;
    private @NotNull TelescopingArm telescopingArm;
    private @NotNull SystemCoordinator systemCoordinator;
    private @NotNull Grabber grabber;

    private @NotNull Controller xbox;
    private @NotNull Controller buttonPanel;

    // Autonomous
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final SendableChooser<String> sideChooser = new SendableChooser<>();

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        drive = Drive.getInstance();
        robotTracker = RobotTracker.getInstance();
        visionHandler = VisionHandler.getInstance();
        elevator = Elevator.getInstance();
        telescopingArm = TelescopingArm.getInstance();
        grabber = Grabber.getInstance();
        systemCoordinator = SystemCoordinator.getInstance();
        xbox = new Controller(0);
        buttonPanel = new Controller(2);

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

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during
     * disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }


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
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        buttonPanel.update();
        xbox.update();
        drive.swerveDriveFieldRelative(getControllerDriveInputs());

        if (xbox.getRisingEdge(XboxButtons.A)) {
            robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), new Rotation2d()));
        }
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

        if(xbox.getRisingEdge(XboxButtons.X)){
            elevator.elevatorStallIntoBottom();
        } else if(xbox.getRisingEdge(XboxButtons.Y)){
            telescopingArm.telescopingArmStallIntoBottom();
        }
    }


    public void startSubsystems() {
        drive.start();
        robotTracker.start();
        visionHandler.start();
        grabber.start();
        elevator.start();
        telescopingArm.start();
        systemCoordinator.start();
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), xbox.getRawAxis(4))
                    .applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), xbox.getRawAxis(4))
                    .applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs();
        }
    }
}
