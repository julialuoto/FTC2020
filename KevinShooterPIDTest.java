

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;


@Autonomous(name="KevinShooterPIDTest", group="Linear Opmode")

public class KevinShooterPIDTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive = null;
    private Servo armServo = null;
    private Servo shooterServo = null;
    private DcMotor intakeDrive = null;
    private DcMotorEx shooterDrive = null;
    private int singleStep =1;
    
    
     public void waitFor(int ms){
        telemetry.addData("waiting ", ms);
            telemetry.update();
            
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
         while(timer.milliseconds()< ms){
                if(!opModeIsActive()) return;
                telemetry.addData("shooter",shooterDrive.getVelocity(AngleUnit.DEGREES));
                telemetry.update();
            }
            telemetry.addData("done",ms);
            telemetry.update();
    }
    public void shoot(int howMany){
        while(howMany > 0){
            telemetry.addData("shooting ", howMany);
            telemetry.update();
            
            shooterServo.setPosition(.9);
            waitFor(200);
            shooterServo.setPosition(.2);
            waitFor(400);
            howMany--;
        }
    }
    public void driveTo(int left, int right, double speed){
            leftDrive.setTargetPosition(left);
            rightDrive.setTargetPosition(right);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
            while(opModeIsActive() && leftDrive.isBusy()){
                telemetry.addData("driveTo ", leftDrive.getCurrentPosition());
                telemetry.addData("driveTo ", rightDrive.getCurrentPosition());
                telemetry.update();
            }
    }
    public void moveArmTo(int position, double speed)
    {
        armDrive.setTargetPosition(position);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(speed);
            while(opModeIsActive() && armDrive.isBusy()){
                telemetry.addData("driveTo ", armDrive.getCurrentPosition());
                telemetry.update();
            }
    }
    @Override
    public void runOpMode() {
        
        
        boolean goFast = true;
        boolean buttonDown = false;
        boolean goFast2 = true;
        boolean buttonDown2 = false;
        double setShooterSpeed=0;
        int count = 0;
        boolean firstTimeInStop = true;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "RightMotor");
        armDrive = hardwareMap.get(DcMotor.class, "ArmMotor");
        armServo = hardwareMap.get(Servo.class, "Arm");
        shooterServo = hardwareMap.get(Servo.class, "Shooter");
        shooterDrive = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        intakeDrive = hardwareMap.get(DcMotor.class, "IntakeMotor");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        PIDFCoefficients cof = shooterDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);        
        cof.p = 60;        
        cof.i = 18;        
        shooterDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,cof);
        
        shooterServo.setPosition(.2);
        armServo.setPosition(0);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double armPower;
            
            int targetMode = 1;
            
            //read camera
            
            shooterDrive.setVelocity(-200,AngleUnit.DEGREES);
    
           
        
            waitFor(4000);
            shoot(3);
            shooterDrive.setVelocity(0);
            
            
            
            
            telemetry.addData("driveTo ", leftDrive.getCurrentPosition());
            telemetry.addData("driveTo ", rightDrive.getCurrentPosition());
            telemetry.update();
            return;
            
        }
    }
    
}
