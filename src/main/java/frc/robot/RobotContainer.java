// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDriveAdv;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ArmAnglerCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ArmAnglerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.TunableNumber;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.jsontype.impl.AsPropertyTypeDeserializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

                                                            

                                                                
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(1);
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController secondaryDriverXbox =
      new CommandXboxController(OperatorConstants.kSecondaryDriverControllerPort);

  TunableNumber angleP = new TunableNumber("Swerve/PID/ModuleAngle/P", SwerveParser.pidfPropertiesJson.angle.p);
  TunableNumber angleD = new TunableNumber("Swerve/PID/ModuleAngle/D", SwerveParser.pidfPropertiesJson.angle.d);
  TunableNumber driveP = new TunableNumber("Swerve/PID/ModuleDrive/P", SwerveParser.pidfPropertiesJson.drive.p);
  TunableNumber driveD = new TunableNumber("Swerve/PID/ModuleDrive/D", SwerveParser.pidfPropertiesJson.drive.d);

  private SendableChooser<Command> autoChooser = null;

  // putting these here
   public final ArmAnglerSubsystem m_ArmAngler = new ArmAnglerSubsystem();
   public final IntakeSubsystem m_Intake = new IntakeSubsystem();
   public final ShootSubsystem m_Shoot = new ShootSubsystem();
   // public final ClimberSubsystem m_Climber = new ClimberSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();


     AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

                                                                   

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND)
    ).withName("driveFieldOrientedDirectAngle");
    SmartDashboard.putData(driveFieldOrientedDirectAngle);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND)
    ).withName("driveFieldOrientedAnglularVelocity");
    SmartDashboard.putData(driveFieldOrientedAnglularVelocity);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND)
    ).withName("driveFieldOrientedDirectAngleSim");

    Command driveRobotOriented = drivebase.driveCommandRobotRelative(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    driveRobotOriented.setName("driveRobotOriented");
    SmartDashboard.putData(driveRobotOriented);

    Command zeroGyro = drivebase.runOnce(() -> drivebase.zeroGyro()).withName("zeroGyro");
    // Command zeroGyro = new InstantCommand(drivebase::zeroGyro, drivebase).withName("zeroGyro");
    SmartDashboard.putData(zeroGyro);

    Command resetOdometrytoAllianceZero = drivebase.runOnce(
      () -> drivebase.resetOdometry(drivebase.invertIfFieldFlipped(new Pose2d(0, 0, new Rotation2d())))
    ).withName("resetOdometrytoAllianceZero");
    SmartDashboard.putData(resetOdometrytoAllianceZero);

    Command addFakeVisionReading = drivebase.runOnce(() -> drivebase.addFakeVisionReading())
      .withName("addFakeVisionReading");
    SmartDashboard.putData(addFakeVisionReading);

    Command testMotors = drivebase.run(() -> {
      // SwerveDriveTest.angleModules(drivebase.swerveDrive, new Rotation2d(driverXbox.getLeftX() * Math.PI));
      SwerveDriveTest.powerAngleMotorsDutyCycle(drivebase.swerveDrive, driverXbox.getLeftX());
      SwerveDriveTest.powerDriveMotorsDutyCycle(drivebase.swerveDrive, driverXbox.getLeftY());
    }).withName("testMotors");    
    SmartDashboard.putData(testMotors);

    Command testAngleMotors = drivebase.run(() -> {
      SwerveDriveTest.angleModules(drivebase.swerveDrive, new Rotation2d(driverXbox.getLeftX() * Math.PI));
    }).withName("testAngleMotors");    
    SmartDashboard.putData(testAngleMotors);

    TunableNumber angle = new TunableNumber("testAngle", 90);
    Command testSetAngle = drivebase.runEnd(
      () -> SwerveDriveTest.angleModules(drivebase.swerveDrive, Rotation2d.fromDegrees(angle.get())),
      () -> SwerveDriveTest.angleModules(drivebase.swerveDrive, Rotation2d.fromDegrees(0))
    ).withName("testSetAngle");    
    SmartDashboard.putData(testSetAngle);

    /* 
    Command testDriveToPose = drivebase.runOnce(
      () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).andThen(
      drivebase.driveToPose(
        new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0))))
      .withName("testDriveToPose");
      */
    Command testDriveToPose = drivebase.driveToRelativePose(new Pose2d(new Translation2d(1, 1),
                                  Rotation2d.fromDegrees(0)))
      /*.asProxy()*/.withName("testDriveToPose");
    SmartDashboard.putData(testDriveToPose);

    drivebase.setDefaultCommand(
      // testMotors);
      // driveRobotOriented);
      driveFieldOrientedDirectAngle);
      // driveFieldOrientedAnglularVelocity);
      // !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    //buttons    System.out.println("got here");
    driverXbox.a().whileTrue(new ArmAnglerCommand(m_ArmAngler, () -> 1));
    driverXbox.x().whileTrue(new ArmAnglerCommand(m_ArmAngler, () -> -1));
   
    m_ArmAngler.setDefaultCommand(new ArmAnglerCommand(
      m_ArmAngler,
      () -> (secondaryDriverXbox.getLeftTriggerAxis() - secondaryDriverXbox.getRightTriggerAxis())));


    secondaryDriverXbox.a().onTrue(new IntakeCommand(m_Intake, 0.5).withTimeout(1));
    secondaryDriverXbox.b().onTrue(new IntakeCommand(m_Intake, -0.5).withTimeout(2));
    //secondaryDriverXbox.y().whileTrue(new ShootCommand(m_Shoot));

     secondaryDriverXbox.y().onTrue(new ParallelCommandGroup(
         new ShootCommand(m_Shoot),
         new IntakeCommand(m_Intake, 1)
     ).withTimeout(2));

    /* 
    driverXbox.b().whileTrue(new IntakeCommand(m_Intake));
    driverXbox.y().whileTrue(new ParallelCommandGroup(
        new ShootCommand(m_Shoot),
        new IntakeCommand(m_Intake)
    ));
    driverXbox.leftBumper().whileTrue(new ClimberCommand(m_Climber, 1));
    driverXbox.rightBumper().whileTrue(new ClimberCommand(m_Climber, -1));
    */
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(drivebase);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("joystick/left-X", driverXbox.getLeftX());
    SmartDashboard.putNumber("joystick/left-Y", driverXbox.getLeftY());
    SmartDashboard.putNumber("joystick/right-X", driverXbox.getRightX());
    SmartDashboard.putNumber("joystick/right-Y", driverXbox.getRightY());

    SmartDashboard.putNumber("pose/x", drivebase.getPose().getX());
    SmartDashboard.putNumber("pose/y", drivebase.getPose().getY());
    SmartDashboard.putNumber("pose/z", drivebase.getPose().getRotation().getDegrees());

    SmartDashboard.putString("alliance", drivebase.isFieldFlipped() ? "RED" : "BLUE");

    if (angleD.hasChanged() || angleP.hasChanged()) {
      for (SwerveModule module : drivebase.swerveDrive.getModules()) {
        module.getAngleMotor().configurePIDF(new PIDFConfig(angleP.get(), angleD.get()));
      }
    }
    if (driveD.hasChanged() || driveP.hasChanged()) {
      for (SwerveModule module : drivebase.swerveDrive.getModules()) {
        module.getDriveMotor().configurePIDF(new PIDFConfig(driveP.get(), driveD.get()));
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */  
  private void configureBindings()
  {

    /*
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // return new PathPlannerAuto("test auto");
    return autoChooser.getSelected();
    // return drivebase.getAutonomousCommand("small path");

    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}