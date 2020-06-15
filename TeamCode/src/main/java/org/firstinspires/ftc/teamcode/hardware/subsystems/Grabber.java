package org.firstinspires.ftc.teamcode.hardware.subsystems;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.SimpleServo;
public class Grabber implements Subsystem{
    private SimpleServo grabber;
    public Grabber(SimpleServo servo){
        grabber = servo;
        initialize();
    }
    @Override
    public void initialize() {
        grabber.setInverted(false);
    }

    @Override
    public void reset() {
        grabber.setPosition(0.0);
    }
    public void grab() {
        grabber.setPosition(0.7);
    }
    public void release(){
        grabber.setPosition(0.3);
    }

    @Override
    public void loop() {
        grabber.setPosition(0.0);
    }

    @Override
    public void stop() {
        grabber.setPosition(grabber.getPosition());
        reset();
    }

    @Override
    public void disable() {
        grabber.disable();

    }
}
