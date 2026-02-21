/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc4388.utility.DeferredBlock;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.controller.ButtonBox;
import frc4388.utility.controller.DeadbandedXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// Autos
import frc4388.utility.controller.VirtualController;
import frc4388.utility.controller.XboxController;
import frc4388.robot.commands.MoveForTimeCommand;
import frc4388.robot.commands.alignment.RotTo45;
import frc4388.robot.commands.MoveUntilSuply;
// import frc4388.robot.commands.alignment.DriveToReef;
// import frc4388.robot.commands.wait.waitElevatorRefrence;
// import frc4388.robot.commands.wait.waitEndefectorRefrence;
// import frc4388.robot.commands.wait.waitFeedCoral;
import frc4388.robot.commands.wait.waitSupplier;
import frc4388.robot.constants.Constants;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.constants.Constants.OIConstants;
import frc4388.robot.constants.Constants.SimConstants.Mode;

import com.pathplanner.lib.commands.PathPlannerAuto;

// Subsystems
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (2including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    
    public final RobotMap m_robotMap = new RobotMap(Mode.REAL);
    
    /* Subsystems */
    public final LED m_robotLED = new LED();
    public final Vision m_vision = new Vision();
    // public final Elevator m_robotElevator = new Elevator(m_robotMap.elevatorIO, m_robotLED);
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain, m_vision);
    // public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain);

    // public final LiDAR reefLidar = new LiDAR(m_robotMap.reefLidar, "Reef");
    // public final LiDAR reverseLidar = new LiDAR(m_robotMap.reverseLidar, "Reverse");


    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
    // private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTONBOX_ID);

    // public List<Subsystem> subsystems = new ArrayList<>();

    // ! Teleop Commands
    public void stop() {
        new InstantCommand(()->{}, m_robotSwerveDrive).schedule();
        m_robotSwerveDrive.stopModules();
        Constants.AutoConstants.Y_OFFSET_TRIM.set(0);
    }

    // ! /*  Autos */
    private SendableChooser<String> autoChooser;
    private Command autoCommand;


    public RobotContainer() {
        
        configureButtonBindings();
        
        DeferredBlock.addBlock(() -> { // Called on first robot enable
            m_robotSwerveDrive.resetGyro();
        }, false);
        DeferredBlock.addBlock(() -> { // Called on every robot enable
            TimesNegativeOne.update();
        }, true);


        DriverStation.silenceJoystickConnectionWarning(true);

        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {

            // IF the driver is holding the aim button, aim the robot towards the hub
            if(m_driverXbox.getRightTriggerAxis() > 0.5) {
                // Aim 
                Translation2d shootTarget = new Translation2d();
                // new Rotation2()
                Rotation2d ang = m_robotSwerveDrive.getPose2d().getTranslation().minus(shootTarget).getAngle();
                m_robotSwerveDrive.driveFieldAngle(
                    getDeadbandedDriverController().getLeft(),
                    ang);
            } else {
                // Drive normally
                m_robotSwerveDrive.driveWithInput(
                    getDeadbandedDriverController().getLeft(),
                    getDeadbandedDriverController().getRight(),true);
            }

        }, m_robotSwerveDrive)
        .withName("SwerveDrive DefaultCommand"));
        m_robotSwerveDrive.setToSlow();

    }
    



    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        

        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro()));

            
        // new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
        //     .onTrue(new RotTo45(m_robotSwerveDrive));

            
        new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
            .onTrue(new InstantCommand(() -> {m_robotSwerveDrive.softStop();}, m_robotSwerveDrive)); 

        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.activateLuigiMode()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.deactivateLuigiMode()));

        new Trigger(() -> getDeadbandedDriverController().getLeftTriggerAxis() >= 0.5)
            .whileTrue(new RunCommand(
                () -> {
                    m_robotSwerveDrive.driveIntakeOrientation(
                        getDeadbandedDriverController().getLeft(),
                        getDeadbandedDriverController().getRight()
                        
                    );
                }, m_robotSwerveDrive))
            .onFalse(new InstantCommand(() -> {
                m_robotSwerveDrive.softStop();
            }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {


        //return autoPlayback;
        //return new GotoPositionCommand(m_robotSwerveDrive, m_vision)
        //return autoChooser.getSelected();
	// try{
	// //     // Load the path you want to follow using its name in the GUI
    //     return autoCommand;
	// } catch (Exception e) {
	//     DriverStation.reportError("Path planner error: " + e.getMessage(), e.getStackTrace());
	    return autoCommand;
	// }
    // return new PathPlannerAuto("Line-up-no-arm");
	// zach told me to do the below comment
	//return new GotoPositionCommand(m_robotSwerveDrive, m_vision);
      //  return new GotoPositionCommand(m_robotSwerveDrive, m_vision, AutoConstants.targetpos);
    }

    public boolean autoChooserUpdated = false;
    public void makeAutoChooser() {
        autoChooser = new SendableChooser<String>();
        
        File dir;

        if(RobotBase.isReal()) {
            dir = new File("/home/lvuser/deploy/pathplanner/autos/");
        } else {
            // dir = new File("C:\\Users\\Ridgebotics\\Documents\\GitHub\\2025RidgeScape\\src\\main\\deploy\\pathplanner\\autos\\");
            dir = new File("/home/astatin3/Documents/GitHub/2025RidgeScape/src/main/deploy/pathplanner/autos");
        }

        String[] autos = dir.list();

        if(autos == null) return;

        for (String auto : autos) {
            if (auto.endsWith(".auto"))
                autoChooser.addOption(auto.replaceAll(".auto", ""), auto.replaceAll(".auto", ""));
            // System.out.println(auto);
        }

        autoChooser.onChange((filename) -> {
            autoChooserUpdated = true;
            if (filename.equals("Taxi")) {
                autoCommand = new SequentialCommandGroup(
                    new MoveForTimeCommand(m_robotSwerveDrive, 
                        new Translation2d(0, -1), 
                        new Translation2d(), 1000, true
                ), new InstantCommand(()-> {m_robotSwerveDrive.softStop();} , m_robotSwerveDrive));
            } else {
                autoCommand = new PathPlannerAuto(filename);
            }
            System.out.println("Robot Auto Changed " + filename);
        });
        // SmartDashboard.putData(autoChooser);

    }

    /**
     * A button binding for two controllers, preferably an {@link DeadbandedXboxController Xbox Controller} and {@link VirtualController Virtual Xbox Controller}
     * @param joystickA A controller
     * @param joystickB A controller
     * @param buttonNumber The button to bind to
     */
    public Trigger DualJoystickButton(GenericHID joystickA, GenericHID joystickB, int buttonNumber) {
        return new Trigger(() -> (joystickA.getRawButton(buttonNumber) || joystickB.getRawButton(buttonNumber)));
    }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    // public ButtonBox getButtonBox() {
    //     return this.m_buttonBox;
    // }
}
