package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.SkyPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueSkyPrkBridgeDep", group = "SkyPrk")
public class BlueSkyPrkBridgeDep extends MainAutonomous {
    private String className = getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences(className);

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

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                Yes
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Recognize skystone
            playSound("ss_power_up");
            encoderDriveSmooth("left", 0.25);
            encoderDriveSmooth("front", 0.5);
            recognizeTarget("Stone Target");
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDriveSmoothDist("right", travelX);

            armExtend();
            armRaise(armDuration);
            gripRelease(gripDuration);
            encoderDriveSmoothDist("front", travelY, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmoothDist("back", travelY-0.5*inPerBlock);

            rotate(-90, turnPower);
            encoderDriveSmooth("front", 2);
            dropSkystone();
            // 2nd skystone
            encoderDriveSmooth("back", 3);
            rotate(90, turnPower);

            encoderDriveSmoothDist("front", travelY-0.5*inPerBlock, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmoothDist("back", travelY-0.5*inPerBlock);

            rotate(-90, turnPower);
            encoderDriveSmooth("front", 3);
            dropSkystone();
            encoderDriveSmooth("back", 0.625);
            encoderDriveSmooth("right", 0.25);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
