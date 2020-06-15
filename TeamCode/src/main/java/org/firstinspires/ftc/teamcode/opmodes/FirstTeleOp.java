package org.firstinspires.ftc.teamcode.opmodes;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkystoneRobot;


@TeleOp(name="TeleOp Offseason")
public class FirstTeleOp extends LinearOpMode {
    private final double COUNTS_PER_INCH = 307.699557;
    private SkystoneRobot robot = new SkystoneRobot(hardwareMap);
    private GamepadEx controllerOne = new GamepadEx(gamepad1);
    private GamepadEx controllerTwo = new GamepadEx(gamepad2);
    private double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot.start();
        robot.reverseEncoders();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Robot Position", robot.getRobotPose());
            telemetry.addData("X Position", robot.returnXPosition());
            telemetry.addData("Y Position", robot.returnYPosition());
            telemetry.addData("Orientation (Degrees)", robot.orientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeftPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRightPosition());
            telemetry.addData("horizontal encoder position", robot.horizontalPosition());

            telemetry.addData("Thread Active", robot.alive());
            telemetry.update();

            //drive
            robot.drive(
                    controllerOne.getLeftX(),
                    controllerOne.getLeftY(),
                    controllerOne.getRightX(),
                    power,
                    Math.toRadians(robot.IMUHeading())
            );


            //intake
            if (controllerTwo.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.intake();
            }else if (controllerTwo.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.outtake();
            }else {
                robot.stopIntake();
            }
            //power increase/decrease
            if(controllerOne.getButton(GamepadKeys.Button.DPAD_UP)){
                power += 0.05;
            }else if(controllerOne.getButton(GamepadKeys.Button.DPAD_DOWN)){
                power -=0.05;
            }
            //lift
            robot.lift(controllerTwo.getLeftY());

            //block grab
            if(controllerTwo.getButton(GamepadKeys.Button.A)){
                robot.grabBlock();
            }
            else if(controllerTwo.getButton((GamepadKeys.Button.B))) {
                robot.releaseBlock();
            }

            //foundation
            if(controllerOne.getButton(GamepadKeys.Button.A)){
                robot.grabFoundation();
            }
            else if(controllerOne.getButton(GamepadKeys.Button.B)){
                robot.releaseFoundation();
            }
            //arm
            if(controllerTwo.getButton(GamepadKeys.Button.X)){
                robot.armDownPosition();
            }else if(controllerOne.getButton(GamepadKeys.Button.Y)){
                robot.armReleasingPosition();
            }
        }
        robot.disable();
    }
}

/*
Hotness rating
1.Jackson Ikari
2. Srihith BrownBoy
3. Rishab the real Stan
4. Ethan the real Downy
5. Kuunav the Palestinian
 */