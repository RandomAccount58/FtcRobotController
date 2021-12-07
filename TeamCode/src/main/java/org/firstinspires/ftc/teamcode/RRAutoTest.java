package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RRTesting")
public class RRAutoTest extends LinearOpMode{
    SampleMecanumDrive robot = new SampleMecanumDrive(hardwaremap);
    public void runOpMode()
    {
        Trajectory myTrajectory = robot.trajectoryBuilder(new pose2d())
                .strafeRight(10)
                .forward(5)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        robot.followTrajectory(myTrajectory);
    }
}
