// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  // initializing motor controllers
  private final WPI_VictorSPX motorR = new WPI_VictorSPX(1);
  private final WPI_VictorSPX motorIntake = new WPI_VictorSPX(2);
  private final WPI_VictorSPX motorUnused = new WPI_VictorSPX(3);  
  private final WPI_VictorSPX motorBelt = new WPI_VictorSPX(4);
  private final WPI_VictorSPX motorL = new WPI_VictorSPX(5);
  private final WPI_TalonFX motorExternal = new WPI_TalonFX(0); //Falcon 500

  // initializing drive, xbox controller, and gyro
  private final DifferentialDrive robotDrive = new DifferentialDrive(motorR, motorL);
  private final XboxController controller = new XboxController(0);
  private final ADIS16448_IMU adis = new ADIS16448_IMU();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    // Sets motors to brake instead of coast
    motorR.setNeutralMode(NeutralMode.Brake);
    motorL.setNeutralMode(NeutralMode.Brake);
    motorBelt.setNeutralMode(NeutralMode.Brake);
    motorIntake.setNeutralMode(NeutralMode.Brake);
    
    adis.calibrate();
    // CTRE suggested set up commands for Falcon 500
    motorExternal.configFactoryDefault();
    motorExternal.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
    motorExternal.configNeutralDeadband(0.01, 30);
    motorExternal.setSensorPhase(false);
    motorExternal.setInverted(false);
    motorExternal.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    motorExternal.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    motorExternal.configNominalOutputForward(0, 30);
    motorExternal.configNominalOutputReverse(0, 30);
    motorExternal.configPeakOutputForward(1, 30);
    motorExternal.configPeakOutputReverse(-1, 30);
    motorExternal.selectProfileSlot(0, 0);
    motorExternal.setSelectedSensorPosition(0, 0, 30);
    
    // Motion Magic Parameters
    // PID
    motorExternal.config_kF(0, 0, 30);
    motorExternal.config_kP(0, 1, 30);
    motorExternal.config_kI(0, 0.005, 30);
    motorExternal.config_kD(0, 10, 30);
    // Path Planning
    motorExternal.configMotionCruiseVelocity(20000, 30);
    motorExternal.configMotionAcceleration(6000, 30);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    // controller inputs
    double leftStickY = -controller.getLeftY();
    double leftStickX = controller.getLeftX();
    double rightTrig = controller.getLeftTriggerAxis();
    double leftTrig = controller.getRightTriggerAxis();
    double rightStickY = -controller.getRightY();
    
    // motor outputs
    double position = 2048*rightStickY;
    motorExternal.set(TalonFXControlMode.MotionMagic, position);
    robotDrive.arcadeDrive(leftStickY, leftStickX); 
    motorBelt.set(-leftTrig);
    motorIntake.set(-rightTrig);

    // writing to Shuffleboard
    SmartDashboard.putNumber("Left Stick X", Math.round(100*leftStickX));
    SmartDashboard.putNumber("Left Stick Y", Math.round(100*leftStickY));
    SmartDashboard.putNumber("Right Trigger", Math.round(100*rightTrig));
    SmartDashboard.putNumber("Left Trigger", Math.round(100*leftTrig));
    SmartDashboard.putNumber("Right Stick Y", Math.round(100*rightStickY));
    SmartDashboard.putNumber("Acceleration (X)", adis.getAccelX());
    SmartDashboard.putNumber("Acceleration (Y)", adis.getAccelY());
    SmartDashboard.putNumber("Angle", adis.getGyroAngleZ());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}

//testing github