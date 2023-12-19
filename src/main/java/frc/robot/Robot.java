// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// The WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController test = new XboxController(0);
  TalonFX motor1 = new TalonFX(2);
  TalonFX motor2 = new TalonFX(1);
  TalonFX motor3 = new TalonFX(4);
  TalonFX motor4 = new TalonFX(3);
  double maxLinearSpeed = 3;
  double maxAngularSpeed = 2*Math.PI;
  Translation2d m_frontLeftLocation = new Translation2d(0.53/2, 0.575/2);
  Translation2d 
  m_frontRightLocation = new Translation2d(0.53/2, -0.575/2);
  Translation2d m_backLeftLocation = new Translation2d(-0.53/2, 0.575/2);
  Translation2d m_backRightLocation = new Translation2d(-0.53/2, -0.575/2);

  // Define Feedforward and PID controller (with constants)
  double kS = 0.0;
  double kV = 0.0;
  double kA = 0.0;
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  PIDController pid = new PIDController(kP, kI, kD);
  double deadband = 0.1;
  
  
  
  
  AHRS gyro = new AHRS();

  public double zeroWithinDeadband(double speed) {
    if (Math.abs(speed) < deadband) {
      return 0.0;
    }
    return speed;
  }

  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Use controller to get x speed, y speed, turn speed

    double left_x = zeroWithinDeadband(test.getLeftX());
    double left_y = zeroWithinDeadband(test.getLeftY());
    double right_y = zeroWithinDeadband(test.getRightY());

    double radius = 0.07; // Meters

    float robotAngle = gyro.getYaw();
    ChassisSpeeds speeds = new ChassisSpeeds(left_y * maxLinearSpeed, left_x*maxLinearSpeed, right_y * maxAngularSpeed);
    Rotation2d rotation = new Rotation2d(robotAngle * Math.PI / 180);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond; // target velocity based on the joystick
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond; 
    double backRight = wheelSpeeds.rearRightMetersPerSecond;

    StatusSignal<Double> fL = motor1.getVelocity(); // converts current speed of the motor to a StatusSignal double 
    StatusSignal<Double> fR = motor2.getVelocity(); 
    StatusSignal<Double> bL = motor3.getVelocity();
    StatusSignal<Double> bR = motor4.getVelocity();
    // NO LONGER USING THIS
    // motor1.set(-1*0.25*frontLeft/maxLinearSpeed);
    // motor2.set(0.25*frontRight/maxLinearSpeed);
    // motor3.set(-1*0.25*backLeft/maxLinearSpeed);
    // motor4.set(0.25*backRight/maxLinearSpeed);

    // Calculates the feedforward for the desired speed of each wheel (using DC motor feedforward)
    double frontLeftVolts = feedforward.calculate(frontLeft); // currently taking a speed and changing it to a voltage 
    double frontRightVolts = feedforward.calculate(frontRight); 
    double backLeftVolts = feedforward.calculate(backLeft);
    double backRightVolts = feedforward.calculate(backRight);
    // Calculates the PID output for the desired speed of each wheel, referencing the current speed (using PID)
    // ...and adds these two outputs, and send them to the motors
    frontLeftVolts += pid.calculate(fL.getValue() * 2 * Math.PI * radius / 10.71, frontLeft); // frontLeft etc. is target velocity
    frontRightVolts += pid.calculate(fR.getValue() * 2 * Math.PI * radius / 10.71, frontRight);
    backLeftVolts += pid.calculate(bL.getValue() * 2 * Math.PI * radius / 10.71, backLeft);
    backRightVolts += pid.calculate(bR.getValue() * 2 * Math.PI * radius / 10.71, backRight);
   //inputs voltage
    motor1.setVoltage(-1*frontLeftVolts); // some motors turn the opposite direction
    motor2.setVoltage(frontRightVolts);
    motor3.setVoltage(-1*backLeftVolts); // some motors turn the opposite direction
    motor4.setVoltage(backRightVolts); 
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}