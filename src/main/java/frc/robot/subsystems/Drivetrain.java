package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;
import frc.robot.RobotMap;


//this is a test

public class Drivetrain extends SubsystemBase implements ISubsystem {


    private TalonSRX topLeftMotor;
    private TalonSRX topRightMotor;
    private TalonSRX bottomLeftMotor;
    private TalonSRX bottomRightMotor;

    private double P = 0.150;
    private double I = 0.1;
    private double D = 1;
    private double integral, previous_error, setpoint = 0;
    private double rcw = 0;
    private AHRS gyro;
  

    public Drivetrain(){
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

    private double SpeedScale = Constants.DRIVETRAIN_SPEED_SCALE;
    public void driveWithMisery(double leftStick, double rightStick, double rotation){
        if(rotation==0.0){
            this.turnToAngle(gyro.getAngle());
        }
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue);
         topRightMotor.set(ControlMode.PercentOutput, -rightValue);
         bottomRightMotor.set(ControlMode.PercentOutput, -rightValue); 
    }
    public void driveWithMisery(double leftStick, double rightStick, double rotation, double FL, double FR, double BL, double BR){
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue ;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue + FL);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue + BL);
         topRightMotor.set(ControlMode.PercentOutput, -rightValue + FR);
         bottomRightMotor.set(ControlMode.PercentOutput, -rightValue + BR); 
    }

    public void leftWheelsForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    public void rightWheelsForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }
    public void mechanumWHeelRight(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
    }
    public void mechanumWHeelLeft(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    public void turnLeft(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }
    public void turnRight(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }
    public void stopRobot(){
        topLeftMotor.set(ControlMode.PercentOutput, 0);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0);
        topRightMotor.set(ControlMode.PercentOutput, 0);
        bottomRightMotor.set(ControlMode.PercentOutput, 0);
    }
    public void driveForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }
    public void driveBackwards(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }
    public void PID () {
        double error = setpoint - gyro.getAngle();
        this.integral += (error * .02);
        double derivative = (error - this.previous_error) / 0.02;
        this.rcw = ((P * error) + (I * this.integral) + (D * derivative));
    }
    public void turnToAngle(double setpoint) {
        this.setpoint = setpoint;
        PID();
        double rotateSpeed = this.rcw;
        if(Math.abs(rotateSpeed) > 0.5) {
            if(rotateSpeed < 0) {
                rotateSpeed = -0.5;
            } else {
                rotateSpeed = 0.5;
            }
        } else {
            rotateSpeed = this.rcw;
        }
        driveWithMisery(Constants.DRIVE_SPEED, 0, rotateSpeed);
    }
    public boolean ifSetAngle(){
        if (gyro.getAngle()==this.setpoint)
            return true;
        return false;
    }
    public AHRS getGyro(){
        return gyro;
    }
    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putNumber("Front Left Wheel", topLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
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
