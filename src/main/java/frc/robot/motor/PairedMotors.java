package frc.robot.motor;

public class PairedMotors {
    
    public final Motor mainMotor;
    public final EncodedMotor encodedMotor;
    public final Motor slaveMotor;

    public PairedMotors(MotorInfo main, MotorInfo slave) {
        
        this.mainMotor = new Motor(main);
        this.encodedMotor = new EncodedMotor(main);
        this.slaveMotor = new Motor(slave);

    }

    public void Spin(double speed) {
        
        mainMotor.motor.set(speed);
        slaveMotor.motor.set(-speed);

    }

    public double getRawAngle(boolean degreesOrVoltage) {
        
        double rawAngle;

        if (degreesOrVoltage) {
            double degreePosition = encodedMotor.orientationEncoder.getPosition();
            
            rawAngle = degreePosition;
        } else {
            double degreesPerVolt = 360/encodedMotor.info.MAX_ENCODER_VALUE;
            double encoderVoltage = encodedMotor.orientationEncoder.getVoltage();
            double voltsPosition = degreesPerVolt * encoderVoltage;

            rawAngle = voltsPosition;
        }

        return rawAngle;

    }

    public double getAngle() {
        
        double angle = getRawAngle(true) - encodedMotor.info.REFERENCE_ANGLE;
        
        return angle;

    }

}
