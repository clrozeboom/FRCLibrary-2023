// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.hardware;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.WpiEncoder;

/** Add your docs here. */
public class ROMIMotor implements MotorController5010 {
    private static final double kCountsPerRevolution = 1440.0;
    private static final double kWheelDiameterMeter = 0.07;
    private Spark motor;
    private GenericEncoder encoder;

    public ROMIMotor(int port) {
        motor = new Spark(port);
        encoder = new WpiEncoder(new Encoder(4 + 2 * port, 5 + 2 * port));
        encoder.setPositionConversion((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
        encoder.reset();
    }

    public ROMIMotor(int port, GenericEncoder encoder) {
        motor = new Spark(port);
        this.encoder = encoder;
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();
    }

    @Override
    public void stopMotor() {
        motor.set(0);
    }

    @Override
    public MotorController5010 duplicate(int port) {
        return new ROMIMotor(port);
    }

    @Override
    public MotorController5010 setSlewRate(double rate) {
        System.err.println("Warning: Can't set slew rate on a pwm spark motor");
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        System.err.println("Can't use follow on a basic pwm spark motor");
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
        System.err.println("Can't use follow on a basic pwm spark motor");
        return this;
    }

    @Override
    public MotorController5010 invert(boolean inverted) {
        this.motor.setInverted(inverted);
        return this;
    }

    @Override
    public MotorController5010 setCurrentLimit(int limit) {
        System.err.println("Warning: Can't set current limit on a pwm spark motor");
        return this;
    }

    @Override
    public GenericEncoder getMotorEncoder() {
        return encoder;
    }

    @Override
    public GenericEncoder getMotorEncoder(Type sensorType, int countsPerRev) {
        return encoder;
    }

    @Override
    public MotorController getMotor() {
        return this;
    }

    @Override
    public void factoryDefault() {
    }

    @Override
    public void setEncoder(GenericEncoder encoder) {
        this.encoder = encoder;
    }

}
