// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   *
   */
  


  private DigitalInput extendHallEffect;
  

  private MotorController5010 extendMotor;
  private SparkMaxPIDController extendController;
  private MotorModelConstants extendConstants;
  private GenericPID extendPID;
  private RelativeEncoder extendEncoder;
  private SimulatedEncoder extendSimEncoder = new SimulatedEncoder(12, 13);

  //private double KFF = 0.000156;

  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 10.0; // kg

  public static final double kMinElevatorHeight = Units.inchesToMeters(24);
  public static final double kMaxElevatorHeight = Units.inchesToMeters(60);

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse = Units.inchesToMeters(14.8828);
      // 2.0 * Math.PI * kElevatorDrumRadius / 8192;

  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_elevatorMech2d;
  private MechanismLigament2d targetPos2d;

  
  private double currentExtendTarget;

  private ElevatorLevel currentLevel = ElevatorLevel.ground; // Unsure of whether this should be stored in subsystem

  // TODO Implement ElevatorFeefForward
  private ElevatorFeedforward extendFeedforward;
  private ElevatorSim extendSim;  

  private Supplier<Double> pivotAngle; 
  
  public ElevatorSubsystem(MotorController5010 extend, GenericPID extendPID,
       MotorModelConstants extendConstants,
      Mechanism2d mech2d, int extendHallEffectPort, Supplier<Double> pivotAngle) {
    
    this.currentExtendTarget = 0;


    this.extendMotor = extend;
    this.extendMotor.setInverted(true);
    this.extendController = ((CANSparkMax) extend).getPIDController();
    this.extendEncoder = ((CANSparkMax) extend).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature , 8192);
    this.extendEncoder.setPositionConversionFactor(kElevatorEncoderDistPerPulse);
    this.extendPID = extendPID;
    this.extendConstants = extendConstants;

    this.m_mech2d = mech2d;
    m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 5, 20);
    m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Elevator", Units.metersToInches(kMinElevatorHeight), -30.0, 6, new Color8Bit(Color.kOrange)));
    targetPos2d = m_mech2dRoot.append(
      new MechanismLigament2d("Target", Units.metersToInches(kMinElevatorHeight), -30, 6, new Color8Bit(Color.kBlue)));        
    
    extendSim = new ElevatorSim(DCMotor.getNEO(1), 25, 
      kCarriageMass, kElevatorDrumRadius, kMinElevatorHeight, kMaxElevatorHeight, false);

    extendFeedforward = new ElevatorFeedforward(extendConstants.getkS(), extendConstants.getkV(),
        extendConstants.getkA());
    
      
    extendController.setP(extendPID.getkP());
    extendController.setI(extendPID.getkI());
    extendController.setD(extendPID.getkD());
    extendController.setFeedbackDevice(extendEncoder);

    extendController.setFF(0.0);

    extendController.setSmartMotionMaxVelocity(3000, 0);
    extendController.setSmartMotionMinOutputVelocity(0, 0);
    extendController.setSmartMotionMaxAccel(100, 0);


    this.extendHallEffect = new DigitalInput(extendHallEffectPort);
     
    this.pivotAngle = pivotAngle; 

    SmartDashboard.putNumber("Extend P", extendPID.getkP());
    
  }

  public double isCloseToMin() {
    if(getExtendPosition() < 0.1){
      return 0.5;
    }
    return 1;
  }

  public void reset() {

  }

  public void setExtendEncoderPosition(double pos) {
    this.extendEncoder.setPosition(pos);
  }

  

  // public void runExtendToTarget(double position) {
  //   this.currentExtendTarget = position;
  //   SmartDashboard.putNumber("Extend Target", currentExtendTarget);
  //   targetPos2d.setLength(currentExtendTarget);
  //   if (Robot.isReal()) {
  //     extendController.setFF(getFeedFowardVoltage() / currentExtendTarget);
  //     extendController.setReference(this.currentExtendTarget, CANSparkMax.ControlType.kPosition, 0);
  //     SmartDashboard.putNumber("Extend FF", getFeedFowardVoltage());
  //   } else {
  //     extendPow((currentExtendTarget - getExtendPosition()) / kMaxElevatorHeight);
  //   }
  // }

  public void runExtendToTarget(double position){
    this.currentExtendTarget = position;
    double kP = SmartDashboard.getNumber("Extend P", extendPID.getkP());
    double kF = getFeedFowardVoltage(); 
    // double error = (position - getExtendPosition());
    // double signum = Math.signum(error);
    // double pow = signum < 0 ? kP / 2.0 : kP * + kF; 
    double pow = kP * (position - getExtendPosition()) + kF; 
    extendMotor.setVoltage(pow);
  }

  public double getFeedFowardVoltage(){
    return Math.sin(Units.degreesToRadians(pivotAngle.get())) * (.8);
  }

  public double getExtendPosition() {
    if (Robot.isReal()) {
      return extendEncoder.getPosition();
    } else {
      return extendSimEncoder.getPosition();
    }
  }

  

  public boolean isExtendAtTarget() {
    return Math.abs(getExtendPosition() - this.currentExtendTarget) < 0.05;
  }

  

  public double getExtendTarget() {
    return this.currentExtendTarget;
  }

  public boolean isElevatorIn() {
    return !extendHallEffect.get();
  }

  

  

  public ElevatorLevel getElevatorLevel() {
    return this.currentLevel;
  }

  public void setElevatorLevel(ElevatorLevel level) {
    this.currentLevel = level;
    targetPos2d.setLength(currentLevel.getExtensionPosition());
    targetPos2d.setAngle(currentLevel.getPivotPosition());
  }

  

  public void extendPow(double pow) {
    extendMotor.set(pow + ((pow == 0) ? (getFeedFowardVoltage() / 12) : 0));
    
    SmartDashboard.putNumber("Elevate Power", extendMotor.get());
    SmartDashboard.putNumber("Elevate Current", ((CANSparkMax) extendMotor).getOutputCurrent());
    SmartDashboard.putNumber("Extend Position", extendEncoder.getPosition());
  }

  public boolean atMinHardStop(){
    //TODO: Test if encoder is greater
    return isElevatorIn() && extendEncoder.getVelocity() < 0;
  }

  public boolean atMaxHardStop(){
    return getExtendPosition() > 1.82 && extendEncoder.getVelocity() > 0;
  }

  public void stopExtend(){
    extendMotor.set(0);
  }

  public void stopAndHoldExtend(){
    extendMotor.setVoltage(getFeedFowardVoltage());
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      m_elevatorMech2d.setLength(Units.metersToInches(getExtendPosition()));

      if (isElevatorIn()){
        setExtendEncoderPosition(kMinElevatorHeight);
      }

      if(atMaxHardStop()){
        stopAndHoldExtend();
      }

      if(atMinHardStop()){
        stopExtend();
      }

      SmartDashboard.putBoolean("Elevator In: ", isElevatorIn());
    }
    SmartDashboard.putNumber("Motor Pow: ", extendMotor.get());  
    
    SmartDashboard.putNumber("Elevator Position: ", getExtendPosition());
    //SmartDashboard.putNumber("Abs", KFF);

    SmartDashboard.putNumber("Elevator Level", currentLevel.getExtensionPosition());

    SmartDashboard.putNumber("Pivot Level", currentLevel.getPivotPosition());

    SmartDashboard.putNumber("Extend Velocity Encoder", extendEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    extendSim.setInput(extendMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    extendSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    extendSimEncoder.setPosition(extendSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(extendSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(extendSim.getPositionMeters()));
  }
}