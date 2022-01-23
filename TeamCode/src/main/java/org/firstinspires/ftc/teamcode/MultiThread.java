package org.firstinspires.ftc.teamcode;

//Start of multithreading
public class MultiThread implements Runnable{
    private Thread t;
    private String threadName;
    private int height;
    private final int TOLERANCE = 50;
    private RobotDrive robot;
    private boolean exit = false;
    public boolean running = false;

    public MultiThread(String name,RobotDrive r)
    {
        threadName = name;
        robot = r;
    }
    public void run()
    {
        running = true;
        while((robot.liftMotor.getCurrentPosition() < height && robot.liftMotor.getCurrentPosition() > height - TOLERANCE) || !exit)
        {
            if (robot.liftMotor.getCurrentPosition() < height - TOLERANCE)
                robot.liftMotor.setPower(1);
            else if (robot.liftMotor.getCurrentPosition() > height)
                robot.liftMotor.setPower(-1);
        }
        robot.liftMotor.setPower(0);
        running = false;
    }

    public void start()
    {
        if (t == null) {
            t = new Thread (this, threadName);
            t.start ();
        }
    }

    public void terminate() { exit = true; }

    public void changeLevel(int h) { height = h; }
}
