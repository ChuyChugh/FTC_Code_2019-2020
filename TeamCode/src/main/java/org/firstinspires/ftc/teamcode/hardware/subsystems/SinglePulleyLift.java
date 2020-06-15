package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;

public class SinglePulleyLift implements Subsystem {

    private YellowJacketEx m_motor;

    public SinglePulleyLift(YellowJacketEx motor) {
        m_motor = motor;

        initialize();
    }

    @Override
    public void initialize() {
        m_motor.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        m_motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BREAK);
    }

    public void lift(double power) {
        m_motor.pidWrite(power);
    }

    @Override
    public void reset() {
        m_motor.resetController();
        m_motor.resetEncoder();
    }

    @Override
    public void loop() {
        m_motor.set(0.5);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void disable() {
        m_motor.disable();
    }

}
