package ftc.library;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.PIDController;
import ftc.library.MaelstromControl.PIDPackage;
import ftc.library.MaelstromDrivetrains.DrivetrainModels;
import ftc.library.MaelstromDrivetrains.MaelstromDrivetrain;
import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromSensors.MaelstromIMU;
import ftc.library.MaelstromSensors.MaelstromOdometry;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;
import ftc.library.MaelstromWrappers.MaelstromTelemetry;

public abstract class MaelstromRobot implements TimeConstants {
    public abstract void initHardware(HardwareMap hwMap);
    public abstract PIDPackage pidPackage();
    public MaelstromTelemetry feed;
    public MaelstromDrivetrain dt;
    public MaelstromOdometry xPos;
    public MaelstromOdometry yPos;
    public MaelstromIMU imu;
    public MaelstromUtils.AutonomousOpMode auto;
    public double gearRatio;
    public MaelstromController controller;
    private double yDirection;
    private double xDirection;
    private double angle;
    private double fieldCentric;
    private double desiredAngle;
    private double speeds[];
    public PIDController distanceDrive = new PIDController(pidPackage().getDistanceKp(),pidPackage().getDistanceKi(),pidPackage().getDistanceKd(),1);

    public void driveDistance(double distance, double speed, Direction direction, long stopTime){
        dt.eReset();
        distance *= direction.value;
        double counts = distanceToCounts(distance);
        long startTime = System.nanoTime();
        long stopState = 0;
        //double initialHeading = imu.getRelativeYaw();

        while(opModeActive() && stopState <= stopTime){
            double position = dt.getCounts();
            double power = (distanceDrive.power(counts,position))*speed;
            //double power = distanceDrive.power(distance,dt.getInches());

            drive(power);

            feed.add("Power:",power);
            feed.add("Kp*error:",distanceDrive.getP());
            feed.add("Ki*i:",distanceDrive.getI());
            feed.add("Distance:",countsToDistance(dt.getCounts()));
            feed.add("Stop state:",stopState);
            feed.update();

            if(distanceDrive.getError() <= 0.5){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }
    public void driveDistance(double distance){
        driveDistance(distance,0.5,Direction.FORWARD,1000);
    }

    public void rotate(double speed){
        dt.leftDrive.setPower(-speed);
        dt.rightDrive.setPower(speed);
    }
    public void rotateForTime(double speed, double time){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeActive() && (stopState <= time)){
            rotate(speed);
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        }
        stop();
    }

    public void turnAbsolute(double degrees, double speed, Direction direction, long stopTime){
        PIDController turnAngle = new PIDController(pidPackage().getTurnKp(),pidPackage().getTurnKi(),pidPackage().getTurnKd(),1);
        long startTime = System.nanoTime();
        long stopState = 0;
        degrees *= direction.value;
        //double target = imu.getRelativeYaw();

        while (opModeActive() && (stopState <= stopTime)){
            double position = imu.getRelativeYaw();
            double power = (turnAngle.power(degrees,position))*speed;

            dt.setPower(-power,power);

            feed.add("Power:",power);
            feed.add("Kp*error:",turnAngle.getP());
            feed.add("Ki*i:",turnAngle.getI());
            feed.add("Angle:",imu.getRelativeYaw());
            feed.add("Stop state:",stopState);
            feed.update();

            if(turnAngle.getError() <= 0.5){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }
    public void turnAbsolute(double degrees, double speed, Direction direction){
        turnAbsolute(degrees, speed,direction, 0);
    }
    public void turn(double degrees, double speed, Direction direction, long stopTime){
        turnAbsolute(imu.getRelativeYaw() + degrees,speed,direction,stopTime);
    }
    public void turn(double degrees, double speed, Direction direction){
        turnAbsolute(imu.getRelativeYaw() + degrees,speed,direction,0);
    }
    public void driveToPos(int xTarget, int yTarget, double maxSpeed, long stopTime){

        xTarget = xPos.getTargetCounts(xTarget);
        yTarget = yPos.getTargetCounts(yTarget);

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        dt.eReset();
        xPos.reset();
        yPos.reset();

        long startTime = System.nanoTime();
        long stopState = 0;
        //double initialHeading = imu.getRelativeYaw();
        double distance = Math.hypot(xTarget, yTarget);

        while(opModeActive() && (stopState <= stopTime)){
            angle = Math.atan2(xTarget-xPos.getPosition(),yTarget-yPos.getPosition());
            double adjustedAngle = angle + Math.PI;
            //PIDController drivePos = new PIDController(pidPackage().getDrivePosKp(),pidPackage().getDrivePosKi(),pidPackage().getDrivePosKd(),1);

            frontLeftPower = Math.sin(adjustedAngle);
            backLeftPower = Math.cos(adjustedAngle);
            frontRightPower = Math.cos(adjustedAngle);
            backRightPower = Math.sin(adjustedAngle);

            double pidPower = distanceDrive.power(distance,Math.hypot(xPos.getPosition(),yPos.getPosition()));
            speeds[0] = frontLeftPower*pidPower;
            speeds[1] = backLeftPower*pidPower;
            speeds[2] = frontRightPower*pidPower;
            speeds[3] = backRightPower*pidPower;

            MaelstromUtils.normalizeSpeedsToMax(speeds, maxSpeed);

            dt.leftDrive.motor1.setPower(speeds[0]);
            dt.leftDrive.motor2.setPower(speeds[1]);
            dt.rightDrive.motor1.setPower(speeds[2]);
            dt.rightDrive.motor2.setPower(speeds[3]);

            feed.add("X Position:",xPos.trackPosition(xPos.getPosition()));
            feed.add("Y Position:",yPos.trackPosition(yPos.getPosition()));
            feed.add("Distance:",distance);
            feed.add("Stop state:",stopState);
            feed.add("Kp*error:",distanceDrive.getP());
            feed.add("Ki*i:",distanceDrive.getI());
            feed.add("Kd*d:",distanceDrive.getD());
            feed.update();

            if (Math.abs(distance - Math.hypot(xPos.getPosition(), yPos.getPosition())) <= 0.5) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }
        }
        stop();
    }
    public void driveToPos(int xTarget, int yTarget, double maxSpeed){
        driveToPos(xTarget, yTarget, maxSpeed,0);
    }

    public void strafe(double speed, double angle, long stopTime){
        DrivetrainModels model = dt.getModel();
        if(model == DrivetrainModels.MECH_FIELD || model == DrivetrainModels.MECH_ROBOT){
            double frontLeft;
            double backLeft;
            double frontRight;
            double backRight;

            dt.eReset();
            long startTime = System.nanoTime();
            long stopState = 0;

            angle = Math.toRadians(angle);
            double adjustedAngle = angle + Math.PI/4;

            frontLeft = speed * (Math.sin(adjustedAngle));
            backLeft = speed * (Math.cos(adjustedAngle));
            frontRight = speed * (Math.cos(adjustedAngle));
            backRight = speed * (Math.sin(adjustedAngle));

            while(opModeActive() && stopState <= stopTime){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                dt.leftDrive.motor1.setPower(frontLeft);
                dt.leftDrive.motor2.setPower(backLeft);
                dt.rightDrive.motor1.setPower(frontRight);
                dt.rightDrive.motor2.setPower(backRight);

                feed.add("frontLeft:",frontLeft);
                feed.add("backLeft:",backLeft);
                feed.add("frontRight:",frontRight);
                feed.add("backRights:",backRight);
                feed.add("IMU:",imu.getRelativeYaw());
                feed.add("Stop State:",stopState);
            }
            stop();
        }
        else {
            feed.add("CANNOT RUN: INCORRECT DRIVETRAIN MODEL");
            feed.update();
        }
    }
    public void strafe(double speed, double angle){
        strafe(speed, angle,0);
    }

    public void stop(){
        dt.setPower(0);
    }

    public void drive(double power){
        dt.setPower(power);
    }

    public void drive(double left, double right){
        dt.setPower(left,right);
    }

    public void drive(MaelstromController controller, double speedMultiplier){
        DrivetrainModels model = dt.getModel();
        if(model == DrivetrainModels.ARCADE){
            this.controller = controller;
            yDirection = speedMultiplier * controller.left_stick_y;
            xDirection = speedMultiplier * controller.right_stick_x;

            double left = yDirection - xDirection;
            double right = yDirection + xDirection;

            drive(left,right);
        }
        else if(model == DrivetrainModels.TANK){
            double left = controller.left_stick_y;
            double right = controller.right_stick_y;


            drive(left,right);
        }
        else if(model == DrivetrainModels.MECH_FIELD){

            double leftY = controller.left_stick_y;
            double leftX = controller.right_stick_x;
            double rightX = controller.right_stick_x;

            double x = -leftY;
            double y = leftX;

            double angle = Math.atan2(y,x);
            double fieldCentric = angle + Math.toRadians(imu.getYaw());
            double adjustedAngle = fieldCentric + Math.PI / 4;

            this.angle = angle;
            this.fieldCentric = fieldCentric;

            double speedMagnitude = Math.hypot(x,y);

            if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

            double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

            speeds[0] = (speeds[0] * speedMagnitude * speedMagnitude) - rightX * speedMagnitude;
            speeds[1] = (speeds[1] * speedMagnitude * speedMagnitude) - rightX * speedMagnitude;
            speeds[2] = (speeds[2] * speedMagnitude * speedMagnitude) + rightX * speedMagnitude;
            speeds[3] = (speeds[3] * speedMagnitude * speedMagnitude) + rightX * speedMagnitude;
            this.speeds = speeds;

            dt.leftDrive.motor1.setPower(speeds[0]);
            dt.leftDrive.motor2.setPower(speeds[1]);
            dt.rightDrive.motor1.setPower(speeds[2]);
            dt.rightDrive.motor2.setPower(speeds[3]);
        }
        else if(model == DrivetrainModels.MECH_ROBOT){

            double leftY = controller.left_stick_y;
            double leftX = controller.right_stick_x;
            double rightX = controller.right_stick_x;

            double x = -leftY;
            double y = leftX;

            double angle = Math.atan2(y,x);
            double adjustedAngle = angle + Math.PI / 4;

            this.angle = angle;

            double speedMagnitude = Math.hypot(x,y);

            if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

            double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

            speeds[0] = (speeds[0] * speedMagnitude * speedMagnitude) - rightX * speedMagnitude;
            speeds[1] = (speeds[1] * speedMagnitude * speedMagnitude) - rightX * speedMagnitude;
            speeds[2] = (speeds[2] * speedMagnitude * speedMagnitude) + rightX * speedMagnitude;
            speeds[3] = (speeds[3] * speedMagnitude * speedMagnitude) + rightX * speedMagnitude;
            this.speeds = speeds;

            dt.leftDrive.motor1.setPower(speeds[0]);
            dt.leftDrive.motor2.setPower(speeds[1]);
            dt.rightDrive.motor1.setPower(speeds[2]);
            dt.rightDrive.motor2.setPower(speeds[3]);
        }
    }

    public double distanceToCounts(double distance){
        return (distance/(dt.leftDrive.motor1.getEncoder().getWheelCircumference())*dt.getDriveGearReduction()*dt.leftDrive.motor1.getEncoder().getCPR());
    }

    public double countsToDistance(double counts){
        return (counts*dt.leftDrive.getWheelCircumference()*dt.getDrivenGearReduction())/dt.leftDrive.motor1.getEncoder().getCPR();
    }


    public boolean opModeActive(){return auto.getOpModeIsActive();}

}

