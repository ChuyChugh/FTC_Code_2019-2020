package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SinglePulleyLift;

@TeleOp(name="Sample Robot")
public class SampleRobotTeleOp extends LinearOpMode {

    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MotorGroup m_left, m_right;
    private DifferentialDrive m_drive;

    private GamepadEx driveController = new GamepadEx(gamepad1);
    private GamepadEx mechController = new GamepadEx(gamepad2);

    private YellowJacket435 m_intakeLeft, m_intakeRight;
    private YellowJacketEx m_liftMotor;

    private IntakeTwoWheel m_intake;
    private SinglePulleyLift m_lift;

    @Override
    public void runOpMode() throws InterruptedException {
        m_frontLeft = new YellowJacket435(hardwareMap, "fl");
        m_frontRight = new YellowJacket435(hardwareMap, "fr");
        m_backLeft = new YellowJacket435(hardwareMap, "bl");
        m_backRight = new YellowJacket435(hardwareMap, "br");

        m_intakeLeft = new YellowJacket435(hardwareMap, "il");
        m_intakeRight = new YellowJacket435(hardwareMap, "ir");

        m_liftMotor = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "lift")
        );

        m_left = new MotorGroup(m_frontLeft, m_backLeft);
        m_right = new MotorGroup(m_frontRight, m_backRight);

        m_drive = new DifferentialDrive(m_left, m_right);

        m_lift = new SinglePulleyLift(m_liftMotor);
        m_intake = new IntakeTwoWheel(m_intakeLeft, m_intakeRight);

        waitForStart(); // wait for the play button to be pressed

        while (opModeIsActive() && !isStopRequested()) {
            m_drive.arcadeDrive(
                    driveController.getLeftY(),
                    driveController.getRightX(),
                    true
            );

            if (mechController.getButton(GamepadKeys.Button.LEFT_BUMPER)) m_intake.intake();
            else if (mechController.getButton(GamepadKeys.Button.RIGHT_BUMPER)) m_intake.outtake();
            else m_intake.stop();

            m_lift.lift(mechController.getLeftX());
        }

        m_drive.stopMotor();
        m_lift.disable();
        m_intake.disable();
    }

}
