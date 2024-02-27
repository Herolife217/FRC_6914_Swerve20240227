// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
// import frc.robot.commands.BlindFire;
// import frc.robot.commands.Warning;
// import frc.robot.commands.HoldArm;
// import frc.robot.commands.RunArmClosedLoop;
// import frc.robot.commands.HoldClimber;
// import frc.robot.commands.HomeClimber;
// import frc.robot.commands.RunClimberManual;
// import frc.robot.commands.Feed;
// import frc.robot.commands.HoldIntake;
// import frc.robot.commands.IntakeNoteAutomatic;
// import frc.robot.commands.RunIntakeOpenLoop;
// import frc.robot.commands.AccelerateShooter;
// import frc.robot.commands.RunShooterAtVelocity;
// import frc.robot.commands.ShootNote;


  /*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  public final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kSensorDIOPort);
  public final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  //public final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO);
  //public final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO);

  /* Controllers */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    // The driver's controller
  CommandXboxController operatorXboxController = new CommandXboxController(0);
  CommandXboxController driverXboxController = new CommandXboxController(3);
    
  // driver controls */ 

  /* Driver buttons */
  //private final JoystickButton zeroGyro = new JoystickButton(driverXboxController, XboxController.Button.kY.value);
  //private final JoystickButton robotCentric = new JoystickButton(driverXboxController, XboxController.Button.kA.value);

  // Command Groups

  private final SendableChooser<Command> autoChooser;  

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer(){
   

  configureButtonBindings();

    // Register Named Commands
    // NamedCommands.registerCommand("intake", );
    // NamedCommands.registerCommand("shootSubwoofer", );
    // NamedCommands.registerCommand("shootpodium", );
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    // NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    // NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));



    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    

    // Configure default commands
   
    // Subsystem Default Commands
    m_intake.setDefaultCommand(new HoldIntake(m_intake));
    m_arm.setDefaultCommand(new HoldArm(m_arm));
    m_shooter.setDefaultCommand(new RunShooterAtVelocity(m_shooter, ShooterConstants.kIdleSpeed));
    // m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    // m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.XboxController} or {@link XboxController}), and then calling
   * passing it to a
   * {@link XboxButton}.
   */
  private void configureButtonBindings() {
    // driverXboxController.button(DriverConstants.kSetX)
    //     .whileTrue(new RunCommand(
    //         () -> m_Drive.setX(),
    //         ));

       drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverXboxController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverXboxController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverXboxController.getLeftY(), -driverXboxController.getLeftX()))));

    // reset the field-centric heading on left bumper press / zero gyro
    driverXboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    //driverXboxController.a().onTrue(amp);

    //driverXboxController.button(DriverConstants.kIntake).whileTrue(intake);
    //driverXboxController.button(DriverConstants.kHoldArmDown).toggleOnTrue(new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));

    /*driverXboxController.button(DriverConstants.kSelfDestruct).onTrue(
      new InstantCommand(() -> System.out.println("¡KABOOM!"))
   
    );*/
    
    operatorXboxController.start().whileTrue(new Warning("¡OVERRIDE!"));
    operatorXboxController.rightBumper().whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kManualSpeed));
    //operatorXboxController.rightTrigger().whileTrue(forceReverse);
    operatorXboxController.leftBumper().whileTrue(new RunArmClosedLoop(m_arm, -ArmConstants.kManualSpeed));
    operatorXboxController.leftTrigger().whileTrue(new IntakeNoteAutomatic(m_intake));

    operatorXboxController.a().whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos));
    operatorXboxController.x().whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos));
    //operatorXboxController.y().whileTrue(shootSubwoofer);
    operatorXboxController.b().whileTrue( new RunShooterAtVelocity(m_shooter, ShooterConstants.kSubwooferSpeed));
    
    //operatorXboxController.back().whileTrue(homeClimbers);
    // operatorXboxController.povUp().whileTrue(new RunClimberManual(m_starboardClimber, ClimberConstants.kManualSpeed));
    // operatorXboxController.povDown().whileTrue(new RunClimberManual(m_portClimber, -ClimberConstants.kManualSpeed));
  }
    
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}