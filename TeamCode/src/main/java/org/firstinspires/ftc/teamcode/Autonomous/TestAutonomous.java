package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestAutonomous extends MainAutonomous {
    public String className = getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize hardware
        getHardwareMap();
        initCheck();
        print("Status", "Initialized");
        print("Alliance", teamColor);
        print("Foundation", doFoundation);
        print("Skystone", doSkystone);
        print("Parking Position", parking);
        print("Starting Position", starting);
        telemetry.update();

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            playSound("ss_power_up");
            /*
            timeDrive("front",minPower,1);
            timeDrive("back",minPower,1);
            encoderDrive("front",minPower, 1);
            encoderDrive("back",minPower, 1);
            encoderDriveSmooth("front", 1);
            encoderDriveSmooth("back", 1);

            timeTurn(90, minTurnPower);
            timeTurn(-90, minTurnPower);

            gyroTurn(90,minTurnPower);
            gyroTurn(-90,minTurnPower);

            gyroCurve(90,minTurnPower);
            gyroTurn(90,minTurnPower);
            gyroTurn(90,minTurnPower);
            gyroCurve(-90,minTurnPower);

            hookOn();
            hookOff();

            captureLeftSkystone();
            captureRightSkystone();
            collapseSkystone();

            armExtend();
            armRaise(500);
            gripRelease(150);
            armDrop(500);
            gripHold(150);
            stopAllMotors();
            armCollapse();
            */
            //encoderDriveSmooth("left", 1, drivePower);
            //rotate(180,turnPower);
            //rotate(90, turnPower);
            //rotate(-90, turnPower);
            //encoderDriveSmoothDist("left", 12);
            rotate(90, turnPower);
            rotate(-90, turnPower);
            rotate(-15, turnPower);
            //armExtend();
            //armRaise(50);
            //gripRelease(50);
            //encoderDriveSmooth("front", 0.25, drivePower);
            //armDrop(50);
            //gripHold(50);
            //armRaise(50);
            //curve(-90, turnPower);
            //rotate(-90, turnPower);
            //curve(-90, turnPower);
            //rotate(90, turnPower);
            //encoderDriveSmoothDist("right", travelX);
            //drive("back", 2, drivePower);
            //drive("left", 1, drivePower);
            //drive("right", 1, drivePower);
            /*
            recognizeTarget("Stone Target");
            recognizeSkystone("blue");
             */
            //grabFoundation("blue");

            stopAllMotors();
            visionTargets.deactivate();
        }
    }
}
