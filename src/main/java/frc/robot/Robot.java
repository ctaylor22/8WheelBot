/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.CANPIDController;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax spark_DT_right_1;
  CANSparkMax spark_DT_right_2;
  CANSparkMax spark_DT_right_3;
  
  CANSparkMax spark_DT_left_1;
  CANSparkMax spark_DT_left_2;
  CANSparkMax spark_DT_left_3;
  
  CANPIDController spark_DT_right_PID;
  CANPIDController spark_DT_left_PID;

  XboxController control;

  CANEncoder spark_DT_right_1_enc;
  CANEncoder spark_DT_left_1_enc;

  boolean autoForward = false;

  double DTkP = 0.00003;
  double DTkI = 0;
  double DTkD = 0.00025;
  int DTmaxVel = 1000; // shgould be RPM
  double DTkF = 0.0002; // pid_output + DTkF * reference
  //int DTcurrentLimit = 50;
  double voltage_comp = 11;

  /** this is to test the swerve module */
  /*
  CANSparkMax motor_drive;
  CANEncoder motor_drive_encoder;
  CANPIDController motor_drive_PIDcontroller;
  */

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
    
    spark_DT_right_1 = new CANSparkMax(4, MotorType.kBrushless);
    spark_DT_right_2 = new CANSparkMax(5, MotorType.kBrushless);
    spark_DT_right_3 = new CANSparkMax(6, MotorType.kBrushless);

    spark_DT_left_1 = new CANSparkMax(1, MotorType.kBrushless);
    spark_DT_left_2 = new CANSparkMax(2, MotorType.kBrushless);
    spark_DT_left_3 = new CANSparkMax(3 , MotorType.kBrushless);
    
    spark_DT_left_1.setInverted(true);
    spark_DT_left_1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_left_1.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_left_1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_left_1.enableVoltageCompensation(voltage_comp);

    spark_DT_left_2.follow(spark_DT_left_1);
    spark_DT_left_2.getEncoder();
    spark_DT_left_2.setInverted(false);
    spark_DT_left_2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_left_2.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_left_2.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_left_2.enableVoltageCompensation(voltage_comp);

    spark_DT_left_3.follow(spark_DT_left_1);
    spark_DT_left_3.getEncoder();
    spark_DT_left_3.setInverted(false);
    spark_DT_left_3.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_left_3.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_left_3.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_left_3.enableVoltageCompensation(voltage_comp);

    spark_DT_right_1.setInverted(false);
    spark_DT_right_1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_right_1.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_right_1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_right_1.enableVoltageCompensation(voltage_comp);

    spark_DT_right_2.follow(spark_DT_right_1);
    spark_DT_right_2.getEncoder();
    spark_DT_right_2.setInverted(false);
    spark_DT_right_2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_right_2.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_right_2.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_right_2.enableVoltageCompensation(voltage_comp);

    spark_DT_right_3.follow(spark_DT_right_1);
    spark_DT_right_3.getEncoder();
    spark_DT_right_3.setInverted(false);
    spark_DT_right_3.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark_DT_right_3.enableSoftLimit(SoftLimitDirection.kForward, false);
    spark_DT_right_3.enableSoftLimit(SoftLimitDirection.kReverse, false);
    spark_DT_right_3.enableVoltageCompensation(voltage_comp);

    spark_DT_right_1_enc = spark_DT_right_1.getEncoder();
    spark_DT_left_1_enc = spark_DT_left_1.getEncoder();

    spark_DT_right_PID = spark_DT_right_1.getPIDController();
    spark_DT_right_PID.setP(DTkP);
    spark_DT_right_PID.setI(DTkI);
    spark_DT_right_PID.setD(DTkD);
    spark_DT_right_PID.setFF(DTkF);

    spark_DT_left_PID = spark_DT_left_1.getPIDController();
    spark_DT_left_PID.setP(DTkP);
    spark_DT_left_PID.setI(DTkI);
    spark_DT_left_PID.setD(DTkD);
    spark_DT_left_PID.setFF(DTkF);

    /** swerve module test code */
    /*
    motor_drive = new CANSparkMax(10, MotorType.kBrushless);
    //CANSparkMax motor = new CANSparkMax(99, MotorType.kBrushless);
    //motor_drive.restoreFactoryDefaults(true);
    motor_drive.setMotorType(MotorType.kBrushless);
    motor_drive.setInverted(false);
    motor_drive.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor_drive.enableSoftLimit(SoftLimitDirection.kForward, false);
    motor_drive.enableSoftLimit(SoftLimitDirection.kReverse, false);
    motor_drive.enableVoltageCompensation(11);

    motor_drive_encoder = motor_drive.getEncoder();

    motor_drive_PIDcontroller = new CANPIDController(motor_drive);
    motor_drive_PIDcontroller.setP(DTkP);
    motor_drive_PIDcontroller.setI(DTkI);
    motor_drive_PIDcontroller.setD(DTkD);
    motor_drive_PIDcontroller.setFF(DTkF);
  */

    control = new XboxController(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
    SmartDashboard.putNumber("Right Back Encoder", rB_enc.getPosition());
    SmartDashboard.putNumber("Right Front Encoder", spark_DT_right_enc.getPosition());
    SmartDashboard.putNumber("Left Back Encoder", lB_enc.getPosition());
    SmartDashboard.putNumber("Left Front Encoder", spark_DT_left_enc.getPosition());
    */
    SmartDashboard.putNumber("Drive Left 1 Velocity", spark_DT_left_1_enc.getVelocity());
    SmartDashboard.putNumber("Drive Right 1 Velocity", spark_DT_right_1_enc.getVelocity());
    //SmartDashboard.putNumber("Drive Velocity Conversion Factor", motor_drive_encoder.getVelocityConversionFactor());
    SmartDashboard.putNumber("Drive Left 1 % Output", spark_DT_left_1.getAppliedOutput());
    SmartDashboard.putNumber("Drive Right 1 % Output", spark_DT_right_1.getAppliedOutput());

    SmartDashboard.putNumber("Drive Left 2 % Velocity", spark_DT_left_2.getEncoder().getVelocity());
    SmartDashboard.putNumber("Drive left 2 % Output", spark_DT_left_2.getAppliedOutput());
    SmartDashboard.putBoolean("left drive 2 is follower", spark_DT_left_2.isFollower());
    //SmartDashboard.putNumber("Spark Input Voltage", motor_drive.getBusVoltage());

    SmartDashboard.putNumber("Joystick Left Y", control.getY(Hand.kLeft));
    SmartDashboard.putNumber("Joystick Right X", control.getX(Hand.kRight));
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    super.teleopInit();

    spark_DT_left_1.setIdleMode(IdleMode.kBrake);
    spark_DT_left_2.setIdleMode(IdleMode.kBrake);
    spark_DT_left_3.setIdleMode(IdleMode.kBrake);
    spark_DT_right_1.setIdleMode(IdleMode.kBrake);
    spark_DT_right_2.setIdleMode(IdleMode.kBrake);
    spark_DT_right_3.setIdleMode(IdleMode.kBrake);
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double valueRX;
    double valueLY;

    double DTavgVel = 0.5 * (spark_DT_left_1_enc.getVelocity() + spark_DT_left_1_enc.getVelocity());

    if (Math.abs(control.getRawAxis(4)) < 0.04) {
      valueRX = 0;
    } else {
     valueRX = control.getRawAxis(4);
     valueRX = 0.5 * valueRX*Math.abs(valueRX);
     //valueRX = valueRX * math.abs(valueRX) * (1 - 0.25 * DTavgVel);
    }
    
    if (Math.abs(control.getRawAxis(1)) < 0.04) {
      valueLY = 0;
    } else {
      valueLY = -1*control.getRawAxis(1);
      valueLY = valueLY * Math.abs(valueLY);
    }
    double leftSide = valueLY + valueRX;
    double rightSide = valueLY - valueRX;

    
    //double target_velocity = DTmaxVel * control.getY(Hand.kLeft);
    if (control.getRawAxis(3) > 0.75) {
      spark_DT_left_PID.setReference(DTmaxVel * leftSide, ControlType.kVelocity);
      spark_DT_right_PID.setReference(DTmaxVel * rightSide, ControlType.kVelocity);
    } else {
      spark_DT_left_1.set(0.6* leftSide);
      spark_DT_right_1.set(0.6*rightSide);
    }
    

    /** swerve module test */
    /*
    double target_velocity = DTmaxVel * control.getY(Hand.kLeft);
    if (control.getRawButton(1)) {
      motor_drive_PIDcontroller.setReference(target_velocity, ControlType.kVelocity);
    } else {
      motor_drive.set(control.getY(Hand.kLeft));
    }
    */
    
    SmartDashboard.putNumber("R", rightSide);
    SmartDashboard.putNumber("L", leftSide);
    //SmartDashboard.putNumber("target Velocity", target_velocity);
    
  }

  @Override
  public void testInit() {
    super.testInit();

    spark_DT_left_1.setIdleMode(IdleMode.kCoast);
    spark_DT_left_2.setIdleMode(IdleMode.kCoast);
    spark_DT_left_3.setIdleMode(IdleMode.kCoast);

    spark_DT_right_1.setIdleMode(IdleMode.kCoast);
    spark_DT_right_2.setIdleMode(IdleMode.kCoast);
    spark_DT_right_3.setIdleMode(IdleMode.kCoast);
    //SmartDashboard.putNumber("Spark # to Test", 1);
    ShuffleboardTab shuffle_testtab = Shuffleboard.getTab("Test Mode");
    shuffle_testtab.add("Spark", 1);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    int spark_to_test = 1;
    
    CANSparkMax motor_test;
    switch (spark_to_test) {
      case 1:
        motor_test = spark_DT_left_1;
        break;
      case 2:
        motor_test = spark_DT_left_2;
        break;
      case 3:
        motor_test = spark_DT_left_3;
        break;
      case 4:
        motor_test = spark_DT_right_1;
        break;
      case 5:
        motor_test = spark_DT_right_2;
        break;
      case 6:
        motor_test = spark_DT_right_3;
        break;
      default:
        motor_test = spark_DT_left_1;
        break;
    }

    if (control.getAButton()) {
      motor_test.set(control.getY(Hand.kLeft));
    }
    else {
      motor_test.set(0);
    }
  }
}
