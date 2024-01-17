// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class MathR {
    public static double getDistanceToAngle(double currentDegrees, double desiredDegrees) {
        return modulo(((desiredDegrees) - (currentDegrees)) + 180, 360) - 180;
    }

    public static double modulo(double x, double y) {
        while (x < 0)
            x += y;
        return x % y;
    }

    public static double limit(double input, double min, double max) {
        if (input < min) return min;
        if (input > max) return max;
        return input;
    }

    public static double lerp(double outputMin, double outputMax, double inputMin, double inputMax, double input) {
        return outputMin + (outputMax - outputMin)*(input - inputMin)/(inputMax - inputMin);
    }
}
