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
            /*
            recognizeTarget("Stone Target");
            recognizeSkystone("blue");
             */
            //grabFoundation("blue");
            while (opModeIsActive() && runtime.milliseconds() < 30000) {}
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
