package org.firstinspires.ftc.teamcode.hardware.subsystems;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.SimpleServo;
public class FoundationGrabber implements Subsystem{
    private SimpleServo s_left, s_right;
    public FoundationGrabber(SimpleServo left, SimpleServo right){
        s_left = left;
        s_right = right;
        initialize();
    }

    @Override
    public void initialize() {
        s_left.setInverted(false);
        s_right.setInverted(true);
    }
    /*
    This arm will rotate with servos. It will swing down to the block and then the servos would rotate 180 degrees to go into the foundation part.

     */
    public void grab(){
        s_left.setPosition(0.0);
        s_right.setPosition(0.0);
    }
    public void release(){
        s_left.setPosition(1.0);
        s_right.setPosition(1.0);
    }


    @Override
    public void reset() {
        s_left.setPosition(0.0);
        s_right.setPosition(0.0);
    }

    @Override
    public void loop() {
        s_left.setPosition(0.0);
        s_right.setPosition(0.0);
    }

    @Override
    public void stop() {
        s_left.setPosition(s_left.getPosition());
        s_right.setPosition(s_right.getPosition());
        reset();
    }

    @Override
    public void disable() {
        s_left.disable();
        s_right.disable();
    }
}

