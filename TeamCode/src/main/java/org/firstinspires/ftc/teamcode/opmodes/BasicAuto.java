package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.SkystoneRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="First Autonomous")
public class BasicAuto extends LinearOpMode  {
    private ElapsedTime time = new ElapsedTime();
    private SkystoneRobot robot = new SkystoneRobot(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.setSafetyMode(Safety.SWIFT);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        robot.start();
        robot.reverseEncoders();
        //start of the autonomous. Aim: collect two blocks, put on foundation, set foundation into right place, and finally part into correct position


        /*
        This part of the program first intakes a block then goes back to the first position.
         */
        robot.drive(10 , 24, 1, 90, 0.1, 1);
        robot.intake();
        time.wait(2000);
        robot.stopIntake();
        robot.drive(0, 0, 1, 0, 0.1,1);
        /*
        The second part of this autonomous would go to the foundation and put the block into the foundation.
         */
        robot.drive(-100, 0, 1, 180, 0.1 , 1);
        robot.drive(-100, 48, 1, 0, 0.1, 1);
        robot.drive(-100, 48, 1, 180, 0.1, 1);
        robot.liftToPosition(-3 , 0.25);
        robot.grabBlock();
        robot.liftToPosition(3, 0.25);
        robot.armReleasingPosition();
        robot.releaseBlock();
        robot.armDownPosition();
        /*
        This next part would put the foundation into the zone and then travel back to the original coordinate
         */
        robot.grabFoundation();
        robot.drive(-100, 24, 0.5, 180, 0.1, 1);
        robot.drive(-100, -24, 0.5, 90, 0.1, 1);
        robot.drive(-110, -24, 0.5, 90, 0.1 ,1);
        robot.releaseFoundation();
        robot.drive(0 , 0 , 1, 0, 0.1, 1);
        //This puts the robot back into the same position it was earlier
        robot.drive(-20, 0, 1, 0, 0.1,1);
        /*Second part of autonomous
        This first section of the second part of the autonomous intakes a block and makes the robot go back to the position.
         */
        robot.drive(20, 24 , 1, 90, 0.1, 1);
        robot.intake();
        time.wait(2000);
        robot.stopIntake();
        robot.drive(0, 0, 1, 0, 0.1, 1);
        robot.drive(0,0, 1,90, 0.1 , 1);
        /*
        This section of the program makes the robot drive all the way to the foundation and puts a block in the foundation. After that the autonomous would end after the robot goes under the bridge.
         */
        robot.drive(-110, 0, 1, 90, 0.1, 1);
        robot.liftToPosition(-3, 0.25);
        robot.grabBlock();
        robot.liftToPosition(9, 0.25);
        robot.armReleasingPosition();
        robot.releaseBlock();
        robot.armDownPosition();
        robot.drive(-50, 0, 1, 90, 0.1, 1);




        while(opModeIsActive()){

            telemetry.addData("Robot Position", robot.getRobotPose());
            telemetry.addData("X Position", robot.returnXPosition());
            telemetry.addData("Y Position", robot.returnYPosition());
            telemetry.addData("Orientation (Degrees)", robot.orientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeftPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRightPosition());
            telemetry.addData("horizontal encoder position", robot.horizontalPosition());

            telemetry.addData("Thread Active", robot.alive());
            telemetry.update();
        }
    }
}
