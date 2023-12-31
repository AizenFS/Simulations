// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import javax.xml.transform.Source;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI; // this import
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;





//change the name of the class to "Drive"
//pay attention that the class extends SubsystemBase
public class Drive extends SubsystemBase {
  private final PWMSparkMax leftMotor1 = new PWMSparkMax(1);
  private final PWMSparkMax leftMotor2 = new PWMSparkMax(2);
  private final PWMSparkMax rightMotor1 = new PWMSparkMax(3);
  private final PWMSparkMax rightMotor2 = new PWMSparkMax(4);
  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);
  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  //private final double kDegToMeters = 0.076 * Math.PI ;
  private final MotorControllerGroup m_left = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup m_right = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  //private double factor = (((8.45 * 15.24)/10000))*4.6;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final double start_angel = 148;




  //similar to the init call
  public Drive() {
    rightMotor1 .setInverted(true);
    rightMotor2.setInverted(true);
  }

  
  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      super.simulationPeriodic();
  }
  /*public void setIdleMode(IdleMode mode) {
    leftMotor1.setIdleMode(mode);
  }*/


  @Override
  public void periodic() {
  }
  public void setMotors(double leftSpeed, double rightSpeed) {
    m_left.set(leftSpeed);
    m_right.set(rightSpeed);
    
}
  
  public void resetGyro(){
    gyro.reset();
  }  

  public PWMSparkMax getRightMotor1(){
    return this.rightMotor1;
  }
  public PWMSparkMax getLeftMotor1(){
    return this.leftMotor1;
  }
  
  public void arcadeDrive(double str, double turn){
   
    m_robotDrive.arcadeDrive(str, turn);
  }
  public void tankDrive(double left, double right){
    m_robotDrive.tankDrive(left, right);
  }
  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }
public boolean isYawChanged(){
  if(Math.abs(getYaw()) > 15){   
     return true;
   }
   return false;
}
  public double getYaw(){
    double angle = gyro.getYaw() + start_angel;
    if(angle > 180){
      angle -= 360;
    }
    return angle;
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double getRightEncoder(){
    return rightEncoder.get();
  }
  public double getleftEncoder(){
    return leftEncoder.get();
  }
  public double getBothEncoders(){
    return (getRightEncoder() + getleftEncoder())/2;
  }

  
}
