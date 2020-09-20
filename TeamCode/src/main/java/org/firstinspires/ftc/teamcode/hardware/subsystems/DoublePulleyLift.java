package org.firstinspires.ftc.teamcode.hardware.subsystems;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;


import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;

public class DoublePulleyLift implements Subsystem{
    private YellowJacketEx m_left, m_right;
    private final double COUNTS_PER_INCH = 145.6 / (4 * Math.PI);
    public DoublePulleyLift(YellowJacketEx left, YellowJacketEx right){
        m_left = left;
        m_right = right;

        initialize();
    }

    @Override
    public void initialize() {
        m_left.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        m_left.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BREAK);
        m_left.setInverted(false);
        m_right.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        m_right.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BREAK);
        m_right.setInverted(true);
    }

    @Override
    public void reset() {
        m_left.resetController();
        m_left.resetEncoder();
        m_right.resetController();
        m_right.resetEncoder();
    }
    public void lift(double power) {
        m_left.pidWrite(power);
        m_right.pidWrite(power);
    }
    public void liftToPosition(double position, double power){
        position *= COUNTS_PER_INCH;
        m_left.setTargetPosition(position);
        m_right.setTargetPosition(position);
        m_left.pidWrite(power);
        m_right.pidWrite(power);
    }


    @Override
    public void loop() {
        m_left.set(0.75);
        m_right.set(0.75);
    }

    @Override
    public void stop() {
        m_left.stopMotor();
        m_right.stopMotor();
    }

    @Override
    public void disable() {
        m_left.disable();
        m_right.disable();

    }
}
