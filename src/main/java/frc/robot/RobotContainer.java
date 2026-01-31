// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.swerve.ReduceSwerveTranslationCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import swervelib.SwerveInputStream;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize our controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // final CommandGenericHID operatorController1 = new CommandGenericHID(1);
  // final CommandGenericHID operatorController2 = new CommandGenericHID(2);
   
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/5912_2025"));
    
  private final SendableChooser<Integer> m_autoDelayChooser = new SendableChooser<>();
  private final SendableChooser<String> m_autoPathChooser = new SendableChooser<>();
  private Integer m_selectedDelayAuto;
  private String m_selectedPathAuto;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(SwerveConstants.Deadband)
                                                            .scaleTranslation(SwerveConstants.DefaultScaleTranslation)
                                                            .allianceRelativeControl(true);  

  Command driveFieldOrientedAnglularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands. 
   */
  public RobotContainer() {
    // set our default driving method (field relative)
    driveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // configure our named commands
    configureNamedCommands();

    // configure our auto routines
    configureAutos();

    // configure our controller bindings
    configureBindings();

    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Register named commands before the creation of any PathPlanner Autos or Paths. 
   * It is recommended to do this in RobotContainer, after subsystem 
   * initialization, but before the creation of any other commands.
   */
  public void configureNamedCommands() {
    // NamedCommands.registerCommand("ScoreLevel1Coral", new ScoreLevel1CoralCommand(
    //   driveSubsystem,
    //   elevatorSubsystem,
    //   armSubsystem,
    //   intakeSubsystem
    // ));
  }
  
  /**
   * Register named commands and configure the autonomous command chooser.
   * This will build the auto chooser using the AutoBuilder class, 
   * which pulls in all autos defined in the PathPlanner deploy folder.
   */
  private void configureAutos() {
    m_autoDelayChooser.setDefaultOption( "0 Sec Delay", 0);
    m_autoDelayChooser.addOption( "1 Sec Delay", 1);
    m_autoDelayChooser.addOption( "2 Sec Delay", 2);
    m_autoDelayChooser.addOption( "3 Sec Delay", 3);
    m_autoDelayChooser.addOption( "5 Sec Delay", 5);
    
    m_autoPathChooser.setDefaultOption( "Center10R", "Center10R");
    m_autoPathChooser.addOption( "Center10R-StationRobotRight", "Center10RToStationR");
    m_autoPathChooser.addOption( "Center10R-StationRobotLeft", "Center10RToStationL");
    //m_autoPathChooser.addOption( "Center10R-StationRobotRight-Reef", "Center10RToStationRTo6");
    //m_autoPathChooser.addOption( "Center10R-StationRobotLeft-Reef", "Center10RToStationLTo8");
    m_autoPathChooser.addOption( "OffTheLine", "OffTheLine");
    
    SmartDashboard.putData("Auto-Delay:", m_autoDelayChooser );
    SmartDashboard.putData("Auto-Drive:", m_autoPathChooser ); 
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {   
    // Xbox controller mappings
    driverXbox.start().onTrue((Commands.runOnce(driveSubsystem::resetGyro)));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.b().onTrue(Commands.none());
    driverXbox.x().onTrue(Commands.none());
    driverXbox.rightBumper().onTrue(Commands.none());
    driverXbox.leftBumper().onTrue(Commands.none());
    driverXbox.rightTrigger().whileTrue( new ReduceSwerveTranslationCommand( driveSubsystem, driveAngularVelocity ) );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command delayCommand = null;
       
    double armPostionAuto = ArmConstants.Lvl2Position;

    m_selectedDelayAuto = m_autoDelayChooser.getSelected();
    m_selectedPathAuto = m_autoPathChooser.getSelected();
    
    Command pathCommand;
    
    if( m_selectedDelayAuto > 0 )
      delayCommand = new WaitCommand(m_selectedDelayAuto); 
    
    switch( m_selectedPathAuto )
    {  
      case "Center10R":
        pathCommand = driveSubsystem.getAutonomousCommand("Center10R"); ;
        break;
      case "Center10RToStationR":
        pathCommand = driveSubsystem.getAutonomousCommand("Center10RToStationR"); ;
        break;
      case "Center10RToStationL":
        pathCommand = driveSubsystem.getAutonomousCommand("Center10RToStationL"); ;
        break;
      case "Center10RToStationRTo6":
        pathCommand = driveSubsystem.getAutonomousCommand("Center10RToStationRTo6"); ;
        break;
      case "Center10RToStationLTo8":
        pathCommand = driveSubsystem.getAutonomousCommand("Center10RToStationLTo8"); ;
        break;
      default:
        pathCommand = driveSubsystem.getAutonomousCommand("OffTheLine"); ;
        break;    
    }
   
    if( delayCommand != null && pathCommand != null)
       return delayCommand.andThen(pathCommand);
     else if( pathCommand != null )
       return pathCommand;
    else 
       return Commands.none();
  }

  /**
   * Toggle the motor brake mode
   * @param brake True to brake or false to coast
   */
  public void setMotorBrake(boolean brake) {
    driveSubsystem.setMotorBrake(brake);
  }

  /**
   * Use this to schedule scoring shift feedback based
   * on the game data passed from the DriverStation.
   * @param gameData the game-specific message from the DriverStation
   */
  public void scheduleScoringShiftCommand(char inactiveAlliance) {
    // CommandScheduler.getInstance().schedule(feedbackSubsystem.scoringShiftCommand(inactiveAlliance));
  }
}
