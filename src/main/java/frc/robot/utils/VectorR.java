// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class VectorR implements Cloneable{
    private double x;
    private double y;

    public VectorR(){
        x = 0;
        y = 0;
    }

    public double getAngle(){
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double getMagnitude(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static VectorR fromCartesian(double x, double y){
        VectorR cartVector = new VectorR();
        cartVector.setFromCartesian(x, y);
        return cartVector;
    }

    public void setFromCartesian(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static VectorR fromPolar(double magnitude, double angleDegrees){
        VectorR polarVector = new VectorR();
        polarVector.setFromPolar(magnitude, angleDegrees);
        return polarVector;
    }

    public void setFromPolar(double magnitude, double angleDegrees) {
        x = magnitude * Math.cos(Math.toRadians(angleDegrees));
        y = magnitude * Math.sin(Math.toRadians(angleDegrees));
    }

    public void setMagnitude(double mag) {
        setFromPolar(mag, getAngle());
    }

    public void setAngle(double degrees) {
        setFromPolar(getMagnitude(), degrees);
    }

    public void add(VectorR... vector) {
        for (var vec : vector) {
            x += vec.x;
            y += vec.y;
        }
    }

    public void sub(VectorR vector) {
        x -= vector.x;
        y -= vector.y;
    }

    public void pow(double val) {
        setMagnitude(Math.pow(getMagnitude(), val));
    }

    public void mult(double val) {
        setMagnitude(getMagnitude() * val);
    }

    @Override
    public VectorR clone() {
        return fromCartesian(x, y);
    }

    public static VectorR addVectors(VectorR... vectors) {
        VectorR v3 = new VectorR();
        
        for (VectorR vector : vectors){
            v3.add(vector);
        }
        return v3;
    }

    public static VectorR subVectors(VectorR... vectors) {
        VectorR v3 = vectors[0].clone();
        
        for (int i = 1; i < vectors.length; i++){
            v3.sub(vectors[i]);
        }
        return v3;
    }

    public void rotate(double angle) {
        setAngle(getAngle() + angle);
    }

    public double getTerminalMagnitude() {
        return getMagnitude() * -1;
    }

    public double getTerminalAngle() {
        return getAngle() + 180;
    }
}