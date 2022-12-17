package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Linear OpMode New", group="Linear Opmode")

public class MecanumTeleOp extends LinearOpMode 
{
   private ElapsedTime runtime = new ElapsedTime();
   private DcMotor motorFrontLeft = null;
   private DcMotor motorBackLeft = null;
   private DcMotor motorFrontRight = null;
   private DcMotor motorBackRight = null;
   private DcMotor motorArm = null;
   private DcMotor motorDuckyWheel = null;
   private CRServo servoIntake = null;
   private Servo servoGrabber = null;
   private DigitalChannel limitSwitch = null;
   
   public void ArmMoveTo(int position, double speed)
   {
      motorArm.setTargetPosition(position);
      motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorArm.setPower(speed);
   }
   public void InitArm()
   {
      motorArm.setPower(-0.5); 
      while(!limitSwitch.getState() && opModeIsActive()) ;
      
      motorArm.setPower(0); 
      motorArm.setMode(DcMotor.RunMode.RESET_ENCODERS);
   }
   
   @Override
   public void runOpMode() throws InterruptedException 
   {
      telemetry.addData("Status", "Initialized");
        telemetry.update();
      // Declare our motors
      // Make sure your ID's match your configuration
      motorFrontLeft = hardwareMap.dcMotor.get("LeftFrontMotor");
      motorBackLeft = hardwareMap.dcMotor.get("LeftBackMotor");
      motorFrontRight = hardwareMap.dcMotor.get("RightFrontMotor");
      motorBackRight = hardwareMap.dcMotor.get("RightBackMotor");
      motorArm = hardwareMap.dcMotor.get("ArmMotor");
      motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
      servoIntake = hardwareMap.get(CRServo.class, "IntakeServo");
      servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");
      
      limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
      limitSwitch.setMode(DigitalChannel.Mode.INPUT);          
      
      // Reverse the right side motors
      // Reverse left motors if you are using NeveRests
      motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
      motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
      motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
      motorDuckyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
      
      motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
      telemetry.addData("Arm Position1", "%d", motorArm.getCurrentPosition());
        telemetry.update();
      
      InitArm();
      telemetry.addData("Arm Position2", "%d", motorArm.getCurrentPosition());
        telemetry.update();
        
      waitForStart();
      runtime.reset();

      if (isStopRequested()) return;
      boolean stop = false;

      while (opModeIsActive()) {
         
         ////////////////Mecannon Drive ////////////////////////////
         double y = -gamepad1.left_stick_y; // Remember, this is reversed!
         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x;
         
         double armPower = -gamepad2.right_stick_y / 2;

         // Denominator is the largest motor power (absolute value) or 1
         // This ensures all the powers maintain the same ratio, but only when
         // at least one is out of the range [-1, 1]
         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         double frontLeftPower = (y + x + rx) / denominator;
         double backLeftPower = (y - x + rx) / denominator;
         double frontRightPower = (y + x - rx) / denominator;
         double backRightPower = (y - x - rx) / denominator;
         
         motorFrontLeft.setPower(frontLeftPower);
         motorBackLeft.setPower(backLeftPower);
         motorFrontRight.setPower(frontRightPower);
         motorBackRight.setPower(backRightPower); 
         
         double pos1 = servoGrabber.getPosition();
         telemetry.addData("Test","1, position: 1-%f", pos1);
         telemetry.update();
         
         ///////////////// Grabber ////////////////////
         if (gamepad2.right_bumper == true)//grabber ungrab
         {
            servoGrabber.setPosition(0);
         }
         else //grabber grab
         {
            servoGrabber.setPosition(0.15);
         }
         
         ////////////////////intake//////////////////////
         if (gamepad2.right_trigger >= 0.2) //Makes intake go forward
         {
            servoIntake.setPower(1);
            telemetry.addData("Test","2");
            telemetry.update();
         }
         else if (gamepad2.left_trigger >= 0.2) //Makes intake go rewerse
         {
            servoIntake.setPower(-1);
            
         }
         else 
         {
            servoIntake.setPower(0);
         }
         
         ///////////////// Arm //////////////////////////
         
         if (gamepad2.a == true)// arm moves to the high level
         {
            ArmMoveTo(100,0.5);
         }
         
         if (gamepad2.b == true)// arm moves to the middle level
         {
             ArmMoveTo(-100,0.5);
         }
         if (gamepad2.x == true)//arm moves back to start
         {
            
            motorArm.setPower(0);
         }
         ////////////////////// Ducky Wheel ///////////////////////////
         if (true == gamepad2.y )//Makes ducky wheel move when left drigger pressed
         {
            motorDuckyWheel.setPower(1);
         }
         else 
         {
            motorDuckyWheel.setPower(0);
         }
         
      }
   }
  
}

