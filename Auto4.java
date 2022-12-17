/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Stack;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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
import org.firstinspires.ftc.teamcode.RingLocation;
import org.firstinspires.ftc.teamcode.RingWebCam;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto 4 Use This One", group="Linear Opmode")

public class Auto4 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx armDrive = null;
    private Servo armServo = null;
    private Servo shooterServo = null;
    private DcMotorEx intakeDrive = null;
    private DcMotorEx shooterDrive = null;
    private int singleStep =1;
    private RingWebCam ringWebCam;
    
    // Adding the IMU to use headings
      // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0.0;
    
    private final int STATE_RUN_FORWARD1 = 0;
    private final int STATE_RUN_SHOOT3_A = 1;
    private final int STATE_RUN_SHOOT3_B = 2;
    private final int STATE_RUN_INTAKE =   3;
    private final int STATE_RUN_SHOOT1_A = 4;
    private final int STATE_RUN_SHOOT1_B = 5;
    private final int STATE_RUN_FORWARD2 = 6;
    private final int STATE_STOP         = 7;
    
    public void waitFor(int ms){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
         while(timer.milliseconds()< ms){
               if(!opModeIsActive()) return;
            }
     }
    public void shoot(int howMany){
        while(howMany > 0){
            shooterServo.setPosition(.7);
            waitFor(400);
            shooterServo.setPosition(.0);
            if (--howMany >0 ){
                waitFor(300);
            }
        }
    }
    public void driveForward(int distance, double speed)
    {
        int right, left;
        left = leftDrive.getCurrentPosition();
        right = rightDrive.getCurrentPosition();
        left += distance;
        right += distance;
        driveTo(left,right,speed);
    }
    public void driveReverse(int distance, double speed)
    {
        int right, left;
        left = leftDrive.getCurrentPosition();
        right = rightDrive.getCurrentPosition();
        left -= distance;
        right -= distance;
        driveTo(left,right,speed);
    }
    public void rotateLeft(double angle, double speed)
    {
        PIDRotate(angle,speed);
    }
    public void rotateRight(double angle,double speed)
    {
        PIDRotate(-angle,speed);
    }
    public void turnLeft(int distance, double speed)
    {
        int right, left;
        left = leftDrive.getCurrentPosition();
        right = rightDrive.getCurrentPosition();
        left -= distance;
        right += distance;
        driveTo(left,right,speed);
    }
    public void turnRight(int distance, double speed)
    {
        int right, left;
        
        left = leftDrive.getCurrentPosition();
        right = rightDrive.getCurrentPosition();
        left += distance;
        right -= distance;
        driveTo(left,right,speed);
    }
    public void driveTo(int left, int right, double speed){
            leftDrive.setTargetPosition(left);
            rightDrive.setTargetPosition(right);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setVelocity(2100*speed);
            rightDrive.setVelocity(2100*speed);
            while(opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()){
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
    }
    public void moveArmTo(int position, double speed)
    {
        armDrive.setTargetPosition(position);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(speed);
            while(opModeIsActive() && armDrive.isBusy()){
                //telemetry.addData("moveArmTo ", armDrive.getCurrentPosition());
                //telemetry.update();
            }
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public double deadBand(double A, double B)
    {
        // if absolute value of A is smaller than B, return zero
        if(A >= 0) return (A > B)?A:0.0;
        return (A<=-B)?A:0.0;
    }
    
    public double clampedKValue(double integrated, double K)
    {
        
        if(K == 0.0) return 0.0;
        if((integrated * K) > 1.0) return 1.0 / K;
        if((integrated * K) < -1.0) return -1.0 / K;
        return integrated;
        
    }
    public double absMax(double value, double absMax)
    {
        if(value >= 0) return (value < absMax)?value:absMax;
        return (value<=-absMax)?-absMax:value;
    }
    
    private void newHeading(double degrees, double maxpower, double angleTolerance)
    {
            // Establish constants.
        double Kp = .003;
        double Ki = .001;
        double KmaxiBoost = 0.05;
        double KmaxiError = KmaxiBoost / Ki;
        double basePower = .17;   // Minimum power for turn determined experimentally
        double leftPower, rightPower;
        double currentError;
        double newOutput;
        double iTerm,pTerm;
        double iError = 0;
        
        // Have removed resetAngle();
        // Stop using the encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int count = 0;
   
   //
   // There are two loops here. The outer loop double checks the accuracy of the final
   // heading change. We sometimes overshoot in a bad way. If we are too far off, then 
   // it will go again, likely slower than the first time. 
   //
   
   while(opModeIsActive() && (deadBand((currentError = degrees - getAngle()), angleTolerance) != 0))
        {
            while(opModeIsActive() && (deadBand((currentError = degrees - getAngle()), angleTolerance/2.0) != 0))
            {
                pTerm = absMax(currentError * Kp,1.0); 
                if (Math.abs(pTerm)>=.1){
                    iError = 0.0;
                }
                else
                {
                    iError+=currentError;
                    iError = absMax(iError,KmaxiError);
                }
                iTerm = absMax(iError * Ki,KmaxiBoost);
            
                newOutput = absMax((pTerm + ((pTerm < 0)?-basePower:basePower))+iTerm,maxpower);
            
                leftPower = - newOutput;
                rightPower = newOutput;
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                count++;
            }
        
            // We have reached our goal. Can we stop fast enough to be accurate? 
            leftDrive.setPower(0);
            rightDrive.setPower(0);  
            telemetry.addData("rotate degrees ",degrees);
            telemetry.addData("current globalAngle",globalAngle);
            telemetry.addData("count",count);

            waitFor(100); // Let the motors stop
        
            telemetry.addData("final globalAngle",getAngle());
            telemetry.update();
            
            //
            // We are either good, or we missed a little. So we are Close 
            // Adjust the max power down so we reduce the speed of the turns
            //
            maxpower = maxpower * .6;
        }
        // Turn encoders back on
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // waitFor(1000);
    }
    private void PIDRotate(double degrees, double maxpower)
    {
        // Establish constants.
        double Kp = .003;
        double Ki = .01;
        double basePower = .19;   // Minimum power for turn determined experimentally
        double leftPower, rightPower;
        double currentError;
        double newOutput;
        double iTerm,pTerm;
        double iError = 0;
        
        resetAngle();
        // Stop using the encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int count = 0;
        
   while(opModeIsActive() && (deadBand((currentError = degrees - getAngle()), .2) != 0))
        {
            pTerm = absMax(currentError * Kp,1.0); 
            if (Math.abs(pTerm)>=.2){
                iError = 0.0;
            }
            else{
                iError+=currentError;
            }
            iTerm = absMax(iError * Ki,1.0);
            
            newOutput = absMax((pTerm + ((pTerm < 0)?-basePower:basePower)),maxpower);
            
            leftPower = - newOutput;
            rightPower = newOutput;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("rotate degrees ",degrees);
            telemetry.addData("current globalAngle",globalAngle);
            //telemetry.addData("iError",iError);
            //telemetry.addData("iTerm",iTerm);
            telemetry.addData("pTerm",pTerm);
            telemetry.addData("basePower",basePower);
            telemetry.addData("newOutput",newOutput);
           
            telemetry.update();
            count++;
            
        }
        
        leftDrive.setPower(0);
        rightDrive.setPower(0);  
        telemetry.addData("rotate degrees ",degrees);
        telemetry.addData("current globalAngle",globalAngle);
        telemetry.addData("count",count);
        
         
        // Turn encoders back on
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitFor(100); // Let the motors stop
        
       telemetry.addData("final globalAngle",getAngle());
       telemetry.update();
        // waitFor(1000);
    }
    private void rotateTo(double degrees, double power)
    {
        double  leftPower, rightPower;

        // Turning doesn't use the Encoders. 
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // restart imu movement tracking.
        resetAngle();
        telemetry.addData("rotate to degrees ",degrees);
        telemetry.addData("current globalAngle",globalAngle);
        telemetry.update();
        waitFor(2000);
        
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // wait for rotation to stop.
        waitFor(400);
        getAngle();
        telemetry.addData("rotate to degrees ",degrees);
        telemetry.addData("current globalAngle",globalAngle);
        telemetry.update();
        waitFor(2000);
        // reset angle tracking on new heading.
        resetAngle();
    }
    
    
    @Override
    public void runOpMode() {
        
        int mState = STATE_RUN_FORWARD1;
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
        leftDrive  = hardwareMap.get(DcMotorEx.class, "LeftMotor");
        rightDrive = hardwareMap.get(DcMotorEx.class, "RightMotor");
        armDrive = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armServo = hardwareMap.get(Servo.class, "Arm");
        shooterServo = hardwareMap.get(Servo.class, "Shooter");
        shooterDrive = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        intakeDrive = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        PIDFCoefficients cof = shooterDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);        
        cof.p = 70;        
        cof.i = 18;        
        shooterDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,cof);
        
        shooterServo.setPosition(0.0);
        armServo.setPosition(0);
        moveArmTo(0,.6);        // Insures arm motor is in proper position and mode
        //
        // IMU 
        //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        count = 0;
        while(!imu.initialize(parameters))
        {
            // It failed, say so then try again
            count++;
            telemetry.addData("imu.initialize() failed",count);
            telemetry.addData("imu reports ",imu.getSystemError().toString());
            telemetry.update();
        };
        telemetry.addData("imu.initialize() completed!",count);
        telemetry.update();
     
        /////////////////////////////////////////////////
        /// Camera
        /////////////////////////////////////////////////
        
        RingLocation lastReconizedLocation = new RingLocation();
        
        ringWebCam = new RingWebCam();
        ringWebCam.setHardwareMap(hardwareMap);
        ringWebCam.init();    
        
        while(false == isStarted()) {
            RingLocation location = ringWebCam.findStack(telemetry);
            if(0 != location.mDetected)
            {
                lastReconizedLocation = location;
                runtime.reset();
            }
            else
            {
                if(runtime.milliseconds() > 500)
                {
                    lastReconizedLocation = new RingLocation();
                }
            }
            telemetry.addData("stack", String.format("%d", lastReconizedLocation.mDetected));
            telemetry.update();
        } 
        
        /////////////////////////////////////////////////
        /// Camera End
        /////////////////////////////////////////////////
        
        
        
        
        
        // Wait for the game to start (driver presses PLAY)
        
        
        //waitForStart();
        runtime.reset();
        // Start the logging of measured acceleration
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

/* 
        // Let's do just some turning tests.
        newHeading(-90.0,0.3);
        waitFor(2000);
      
        newHeading(0.0,.3);
        waitFor(2000);
        newHeading(-90.0,.3);
        waitFor(2000);

        newHeading(-180.0,0.4);
        waitFor(2000);
        newHeading(90.0,0.4);
        waitFor(2000);
        newHeading(0.0,0.4);
        waitFor(2000);
*/ 


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double armPower;
            
            int targetMode = lastReconizedLocation.mDetected;
            
            shooterDrive.setVelocity(-210,AngleUnit.DEGREES);
    
            driveForward(900,.4);
            waitFor(1000);
            shoot(3);
            
            switch(targetMode){
                case RingLocation.TARGET_ZERO_RINGS:     //Target A. Second wobble goal scoring currently in progress
                
                    // Drop first wobble goal
                    driveForward(2600,0.6);             
                    newHeading(-90.0,0.4,.6);              
                    driveReverse(400,0.6);              
                                                        
                    moveArmTo(-2400, .6); //drop wobble
                    armServo.setPosition(.8);
                    waitFor(500);
                    
                    // Get other wobble goal
                    driveForward(425,0.6);               
                    newHeading(-1.0,0.4,0.2); 
                    driveReverse(2450,0.6); 
                   
                    //pick up wobble 
                    armServo.setPosition(0);
                    waitFor(800);
                    moveArmTo(-1100, .6);
                    
                    // Drive to target
                    driveForward(1900,0.6); 
                    newHeading(-90.0,0.4,.4);   
                    driveReverse(700,0.6); 
                    
                    // Drop wobble goal
                    moveArmTo(-2300, .6); 
                    armServo.setPosition(.8);
                    waitFor(700);
                    
                    // Be sure we aren't touching: Go fast! 
                    driveForward(400,1.0);
                     
                break;
                case RingLocation.TARGET_ONE_RING:     //Target B
                 // There is one ring out there
                    intakeDrive.setPower(1.0);
                    
                    //turn towards the ring and drive over it
                    newHeading(-10,.4,.8);                              
                    driveForward(1000,.6); 
                  
                    //realign with the goal and shoot
                    newHeading(-2,0.4,.8);
                    waitFor(400);
                    shoot(1);
                   
                    //drive to target
                    driveForward(1500,.7);                          
                    newHeading(173,.4,1.0); 
                    
                    // Deliver wobble goal #1
                    moveArmTo(-2400, .6); 
                    armServo.setPosition(.8);
                    waitFor(700);
                    
                    //drive to wobble goal #2
                    driveForward(3150,.6);  
                    newHeading(90.0,.4,1.0);
                    driveReverse(600,.6);
                    
                    //pick up wobble 
                    armServo.setPosition(0);
                    waitFor(700);
                    moveArmTo(-1100, .6);
                    
                    // Delivery 2nd Wobble Goal
                    newHeading(178.00,.4,.6);
                    driveReverse(3150,.8);
                    
                    //Drive to line. Park.
                    moveArmTo(-2400, .6); 
                    armServo.setPosition(.8);
                    waitFor(600);
                    driveForward(300,.9);
                    
                    intakeDrive.setPower(0); // no longer needed
                   
                    
                  // shooter angle 6 1/8

                break;
                case RingLocation.TARGET_FOUR_RINGS: //Target C, 3 rings
                    intakeDrive.setPower(1);
                    shooterDrive.setVelocity(-200,AngleUnit.DEGREES);
                
                    newHeading(-5,.4,0.6); //turn toward rings
                    driveForward(1000,.8);  
                    newHeading(0,.4,.6);    //turn back towards goal                          
                    driveReverse(100,.8);
                    waitFor(800); //wait is so we can finish loading the rings
                    shoot(3);
                    
                    driveForward(500,.8); // Load remaining rings  
                    
                    // Boogie back to unload wobble goal
                    newHeading(-162,.4,.8);
                    armDrive.setTargetPosition(-1100);
                    driveReverse(2500,.9);                          
                    
                    // Drop wobble goal
                    moveArmTo(-2250, .8); 
                    armServo.setPosition(.8);
                    waitFor(500);
                    
                    // go get 2nd wobble goal
                    driveForward(1750,.8);
                    armDrive.setTargetPosition(-2400);
                    newHeading(0,.5,1.0);
                    driveReverse(1525,.7);
                    
                    //pick up wobble quickly
                    armServo.setPosition(0);
                    waitFor(800);
                    moveArmTo(-2200, .6);  // 
                    armDrive.setTargetPosition(-1100);
                    
                    // Turn around and drive very very fast
                    newHeading(-165,.4,1.0);
                    driveReverse(4000,.9);
                    armServo.setPosition(.8);
                    moveArmTo(-2050, .6); //drop wobble goal
                    
                    // waitFor(600);
                    driveForward(1100,.8);
                    
                    intakeDrive.setPower(0); //intake is no longer necessary
                break;
            }
            
            return;
        }
    }
    
}
