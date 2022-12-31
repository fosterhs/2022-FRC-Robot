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

public class Robot extends TimedRobot {

  // Initializing Motor Controllers
  private final WPI_VictorSPX motorR = new WPI_VictorSPX(1); // Right Side Drive Motor
  private final WPI_VictorSPX motorIntake = new WPI_VictorSPX(2);  // Ball Intake Motor
  private final WPI_VictorSPX motorUnused = new WPI_VictorSPX(3);  // No Motor Connection
  private final WPI_VictorSPX motorBelt = new WPI_VictorSPX(4);  // Belt Motor
  private final WPI_VictorSPX motorL = new WPI_VictorSPX(5);  // Left Side Drive Motor
  private final WPI_TalonFX motorExternal = new WPI_TalonFX(0); // Falcon 500

  // Initializing Drive Object, Controller, and Gyro
  private final DifferentialDrive robotDrive = new DifferentialDrive(motorR, motorL);  // Object for Controlling the Drive Motors
  private final XboxController controller = new XboxController(0);  // Controller
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();  // Gyro
  
  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // Sets Motors to Brake Instead of Coast
    motorR.setNeutralMode(NeutralMode.Brake);
    motorL.setNeutralMode(NeutralMode.Brake);
    motorBelt.setNeutralMode(NeutralMode.Brake);
    motorIntake.setNeutralMode(NeutralMode.Brake);
    
    // Calibrating the Gyro
    gyro.calibrate();

    // CTRE Suggested Setup Commands for Falcon 500
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
    
    // PID Coefficients for Falcon 500
    motorExternal.config_kF(0, 0, 30);
    motorExternal.config_kP(0, 1, 30);
    motorExternal.config_kI(0, 0.005, 30);
    motorExternal.config_kD(0, 10, 30);

    // Motoion Magic Parameters for Falcon 500
    motorExternal.configMotionCruiseVelocity(20000, 30);
    motorExternal.configMotionAcceleration(6000, 30);
  }

  @Override
  public void teleopPeriodic() {

    // Assigning Current XBox Controller State to Variables
    double leftStickY = -controller.getLeftY();
    double leftStickX = controller.getLeftX();
    double rightTrig = controller.getLeftTriggerAxis();
    double leftTrig = controller.getRightTriggerAxis();
    double rightStickY = -controller.getRightY();
    
    // Setting Motors Based on XBox Controller State
    motorExternal.set(TalonFXControlMode.MotionMagic, 2048*rightStickY);  // Falcon 500
    robotDrive.arcadeDrive(leftStickY, leftStickX); // Sets the Two Drive Motors
    motorBelt.set(-leftTrig); // Sets the Belt Motor
    motorIntake.set(-rightTrig); // Sets the Intake Motor

    // Writing Values Shuffleboard
    SmartDashboard.putNumber("Left Stick X", Math.round(100*leftStickX)); 
    SmartDashboard.putNumber("Left Stick Y", Math.round(100*leftStickY));
    SmartDashboard.putNumber("Right Trigger", Math.round(100*rightTrig));
    SmartDashboard.putNumber("Left Trigger", Math.round(100*leftTrig));
    SmartDashboard.putNumber("Right Stick Y", Math.round(100*rightStickY)); 
    SmartDashboard.putNumber("Acceleration (X)", Math.round(gyro.getAccelX())); // Rounded X Acceleration
    SmartDashboard.putNumber("Acceleration (Y)", Math.round(gyro.getAccelY())); // Rounded Y Acceleration
    SmartDashboard.putNumber("Angle", Math.round(gyro.getGyroAngleZ())); // Rounded Angular Position
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
  }
}