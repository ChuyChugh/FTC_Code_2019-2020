package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
@TeleOp(name = "FIRSTONEOFTHEYEAR")
public class niceTele extends LinearOpMode {
    private HardwareMap hardwareMap;
    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MotorGroup m_left, m_right;
    private RevIMU imu;
    private MecanumDrive m_drive;
    private GamepadEx driveController = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {
        m_frontLeft = new YellowJacket435(hardwareMap, "fL");
        m_frontRight = new YellowJacket435(hardwareMap, "fR");
        m_backLeft = new YellowJacket435(hardwareMap, "bL");
        m_backRight = new YellowJacket435(hardwareMap, "bR");

        m_left = new MotorGroup(m_frontLeft, m_frontRight);
        m_right = new MotorGroup(m_backLeft, m_backRight);
        m_drive = new MecanumDrive(m_left, m_right);
        imu = new RevIMU(hardwareMap, "imu");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            m_drive.driveFieldCentric(
                    driveController.getLeftX(),
                    driveController.getLeftY(),
                    driveController.getRightX(),
                    imu.getHeading()
            );
        }
        m_drive.stopMotor();
    }
}
