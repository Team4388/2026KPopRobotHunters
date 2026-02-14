/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4388.robot.commands.MoveForTimeCommand;
import frc4388.robot.constants.Constants;
import frc4388.robot.constants.Constants.OIConstants;
import frc4388.robot.constants.Constants.SimConstants.Mode;
// Subsystems
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.intake.Intake.IntakeMode;
import frc4388.robot.subsystems.shooter.Shooter;
import frc4388.robot.subsystems.shooter.Shooter.ShooterMode;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.DeferredBlock;
import frc4388.utility.compute.FieldPositions;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.controller.DeadbandedXboxController;
// Autos
import frc4388.utility.controller.VirtualController;
import frc4388.utility.controller.XboxController;

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
    
    /*Limit Switch */
    public final DigitalInput m_armLimitSwitch = new DigitalInput(9);

    /* Subsystems */
    public final LED m_robotLED = new LED(Constants.LEDConstants.LED_SPARK_ID);
    //Testing of Colors
    public final Vision m_vision = new Vision();
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain, m_vision);
    public final Intake m_robotIntake = new Intake(m_robotMap.intakeIO, m_armLimitSwitch);
    public final Shooter m_robotShooter = new Shooter(m_robotMap.shooterIO, m_robotSwerveDrive, m_robotIntake, m_robotLED);
    

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

    // private Command RobotShoot = new SequentialCommandGroup(
    //     new InstantCommand(() -> System.out.println(m_robotLED.getMode())),
    //     new InstantCommand(() -> m_robotLED.setMode(LEDPatterns.PARTY_TWINKLES), m_robotLED),
    //     new InstantCommand(() -> System.out.println(m_robotLED.getMode())),
    //     new WaitCommand(5),
    //     new InstantCommand(() -> m_robotLED.setMode(LEDPatterns.SOLID_RED), m_robotLED),
    //     new InstantCommand(() -> System.out.println(m_robotLED.getMode()))
    // );

    // private Command RobotShoot = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotShooter.setMode(Shooter.ShooterMode.Active), m_robotShooter)
    // );

    // private Command RobotIntake = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotIntake.setMode(Intake.IntakeMode.Down), m_robotIntake)
    // );

    public RobotContainer() {
        
        configureButtonBindings();
        
        // Called on first robot enable
        DeferredBlock.addBlock(() -> {
            m_robotSwerveDrive.resetGyro();
        }, false);

        // Called on every robot enable
        DeferredBlock.addBlock(() -> {
            TimesNegativeOne.update();
            FieldPositions.update();
        }, true);


        DriverStation.silenceJoystickConnectionWarning(true);

        // Drive normally
        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
            m_robotSwerveDrive.driveWithInput(
                getDeadbandedDriverController().getLeft(),
                getDeadbandedDriverController().getRight(),true);

        }, m_robotSwerveDrive)
        .withName("SwerveDrive DefaultCommand"));
        
        m_robotSwerveDrive.setToSlow();
        
        makeAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

        // new Trigger(() -> getDeadbandedDriverController().getRightTriggerAxis() >= 0.8 && !operatorManualMode)
        //     .onTrue(RobotShoot)
        //     .onFalse(new InstantCommand(() -> m_robotShooter.setMode(Shooter.ShooterMode.Resting), m_robotShooter));

        // new Trigger(() -> getDeadbandedDriverController().getLeftTriggerAxis() >= 0.8 && !operatorManualMode)
        //     .onTrue(RobotIntake)
        //     .onFalse(new InstantCommand(() -> m_robotIntake.setMode(Intake.IntakeMode.Up), m_robotShooter));
            
        // new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
        //     .onTrue(new InstantCommand(() -> {m_robotSwerveDrive.softStop();}, m_robotSwerveDrive)); 

        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

       
        
        // new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
        //     .onTrue(new InstantCommand(() -> {m_robotShooter.setMode(ShooterMode.Active);}, m_robotShooter)); 
        // new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
        //     .onTrue(new InstantCommand(() -> {m_robotShooter.setMode(ShooterMode.Inactive);}, m_robotShooter)); 
            
        // new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
        //     .onTrue(new InstantCommand(() -> {;}, m_robotIntake)); 
        // new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
        //     .onTrue(new InstantCommand(() -> {m_robotIntake.setMode(IntakeMode.Retracted);}, m_robotIntake)); 



        // new JoystickButton(getDeadbandedDriverController(), XboxController.START_BUTTON)
        //     .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.activateLuigiMode()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(()  -> {
                m_robotIntake.io.updateGains();
                m_robotShooter.io.updateGains();
            }));

            
        // IF the driver is holding the left trigger, intake driving
        new Trigger(() -> getDeadbandedDriverController().getLeftTriggerAxis() >= 0.5)
            .whileTrue(new RunCommand(
                () -> {
                    m_robotSwerveDrive.driveIntake(
                        getDeadbandedDriverController().getLeft(),
                        false
                    );
                }, m_robotSwerveDrive))
            .onTrue(new InstantCommand(() -> {
                m_robotIntake.setMode(IntakeMode.Extended);
            }))
            .onFalse(new InstantCommand(() -> {
                m_robotIntake.setMode(IntakeMode.Retracted);
                m_robotSwerveDrive.softStop();
            }));

        // IF the driver is holding the aim button, aim the robot towards the hub and shooter ready
        new Trigger(() -> getDeadbandedDriverController().getRightTriggerAxis() >= 0.5)
            // .whileTrue(new RunCommand(
            //     () -> {
            //     m_robotSwerveDrive.driveFacingPosition(
            //         getDeadbandedDriverController().getLeft(),
            //         FieldPositions.HUB_POSITION);
            //     }, m_robotSwerveDrive)
            // )
            .onTrue(new InstantCommand(() -> {
                // When Right trigger is pressed, 
                m_robotShooter.setShooterReady();
            }))
            .onFalse(new InstantCommand(() -> {
                // m_robotIntake.setMode(IntakeMode.Retracted);
                m_robotShooter.setShooterNotReady();
            }));


        // D-PAD fine alignment
        new Trigger(() -> getDeadbandedDriverController().getPOV() != -1)
            .whileTrue(new RunCommand(
                () -> m_robotSwerveDrive.driveFine(
                    new Translation2d(
                        1, 
                        Rotation2d.fromDegrees(getDeadbandedDriverController().getPOV())
                    ), 
                    getDeadbandedDriverController().getRight(), 0.15
                ), m_robotSwerveDrive))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.softStop(), m_robotSwerveDrive));
   }

//.onTrue(new InstantCommand(()  -> m_robotLED.setMode(LEDPatterns.SOLID_PINK_HOT)));

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
            dir = new File("C:\\Users\\Ridgebotics\\Documents\\GitHub\\2026KPopRobotHunters\\src\\main\\deploy\\pathplanner\\autos\\");
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
        SmartDashboard.putData(autoChooser);

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