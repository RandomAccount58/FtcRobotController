package org.firstinspires.ftc.teamcode;

//Start of multithreading
public class MultiThread implements Runnable{
    private Thread t;
    private String threadName;
    private int height;
    private final int tolerance = 50;
    private RobotDrive robot;

    public MultiThread(String name, int h,RobotDrive r)
    {
        threadName = name;
        height = h;
        robot = r;
    }
    public void run()
    {
        while(robot.liftMotor.getCurrentPosition() < height && robot.liftMotor.getCurrentPosition() > height - tolerance)
        {
            if (robot.liftMotor.getCurrentPosition() < height - tolerance)
                robot.liftMotor.setPower(1);
            else if (robot.liftMotor.getCurrentPosition() > height)
                robot.liftMotor.setPower(-1);
        }
        robot.liftMotor.setPower(0);
    }
    public void start()
    {
        if (t == null) {
            t = new Thread (this, threadName);
            t.start ();
        }
    }

}
