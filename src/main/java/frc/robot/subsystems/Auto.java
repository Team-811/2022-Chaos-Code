package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vision.Ultrasonic;
public class Auto extends SubsystemBase implements ISubsystem {
    
    private TalonSRX topLeftMotor;
    private TalonSRX topRightMotor;
    private TalonSRX bottomLeftMotor;
    private TalonSRX bottomRightMotor;

    // private double P = 0.150;
    // private double I = 0.1;
    // private double D = 1;
    // private double integral = 0;
    // private double previous_error = 0;
    private double setpoint = 0;
    // private double rcw = 0;
    private ADIS16448_IMU gyro;
    private Ultrasonic ultrasonic;

    public Auto(ADIS16448_IMU m_gyro, Ultrasonic m_ultrasonic){
        gyro = m_gyro;
        ultrasonic = m_ultrasonic;
    resetSubsystem();
        topLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_LEFT );
        topRightMotor= new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_RIGHT );
        bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_LEFT );
        bottomRightMotor= new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_RIGHT );
        topLeftMotor .set(ControlMode.PercentOutput, 0.0f);
        topRightMotor .set(ControlMode.PercentOutput, 0.0f);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomRightMotor.set(ControlMode.PercentOutput, 0.0f);
    }

    private double AutoSpeedScale = Constants.DRIVETRAIN_AUTO_SPEED_SCALE;
    public void autoCrossLine(double speed) {

        double autoForward = speed * AutoSpeedScale;

        topLeftMotor.set(ControlMode.PercentOutput, autoForward); // I highly doubt that this will work
        topRightMotor.set(ControlMode.PercentOutput, autoForward);
        bottomLeftMotor.set(ControlMode.PercentOutput, autoForward);
        bottomRightMotor.set(ControlMode.PercentOutput, autoForward);
        
    }

    public boolean ifSetAngle(){
        if (gyro.getAngle()==this.setpoint)
            return true;
        return false;
    }
    public ADIS16448_IMU getGyro(){
        return gyro;
    }
    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putNumber("Front Left Wheel", topLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Ultrasonic", ultrasonic.getDistance());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

    }

}
