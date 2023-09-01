// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class CheeseStick extends SubsystemBase {
  private MotorController5010 stick;
  private GenericPID PID;
  private Mechanism2d mech2d;
  private SimulatedEncoder stickSimEncoder = new SimulatedEncoder(10, 11);
  private SingleJointedArmSim stickSim;
  private MechanismRoot2d simRoot;
  private MechanismLigament2d simCheeseStick;
  private AbsoluteEncoder stickEncoder;

  /** Creates a new CheeseStick. */
  public CheeseStick(MotorController5010 stick, GenericPID PID, Mechanism2d mech2d) {

    this.stick = stick;
    this.PID = PID;
    this.mech2d = mech2d;
    this.simRoot = mech2d.getRoot("CheeseStick", 50, 50);
    this.simCheeseStick = new MechanismLigament2d("Cheese Stick", 10, 0);
    simRoot.append(simCheeseStick);
    stickEncoder = ((CANSparkMax) stick.getMotor()).getAbsoluteEncoder(Type.kDutyCycle);
    stickEncoder.setPositionConversionFactor(360);

  }

  public double rotateToSetPoint(double position) {
    double current = stickEncoder.getPosition();
    double error = (position - current);
    double power = 0;
    if (Math.signum(error) > 0) {
      power = 0.5;
      if (current > 85) {
        power = 0.25;
      }
    } else {
      power = -0.5;
      if (current < 10) {
        power = 0.25;
      }
    }
    stick.set(power);
    return error;
  }

  public void stop() {
    stick.set(0);
  }

  @Override

  public void periodic() {
    simCheeseStick.setAngle(stickEncoder.getPosition()); // stand in

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    stickSim.setInput(stick.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    stickSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    stickSimEncoder.setPosition(Units.radiansToDegrees(stickSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(stickSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    SmartDashboard.putNumber("Stick Sim Rotation", Units.radiansToDegrees(stickSim.getAngleRads()));
  }
}