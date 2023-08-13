// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors.gyro;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public interface GenericGyro extends Gyro {
    void reset();

    double getAngle();

    double getAngleX();

    double getAngleY();

    double getAngleZ();

    double getRate();

    void setAngle(double angle);
}
