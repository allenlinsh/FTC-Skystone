package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.SkyPrk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

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
            gripRelease(gripDuration/2);
            playSound("ss_power_up");
            encoderDriveSmoothDist("left", fullSkystoneDist+halfSkystoneDist/2.0);
            playSound("ss_roger_roger");
            // 1st skystone

            // grab skystone
            armExtend();
            armRaise(armDuration);
            gripRelease(gripDuration);
            encoderDriveSmooth("front", 1.25, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmooth("back", 0.25);

            // depot to building site
            rotate(-90, turnPower);
            encoderDriveSmooth("front", 1.5);
            dropSkystone();


            // 2nd skystone
            // building site to depot
            encoderDriveSmooth("back", 2);
            rotate(90, turnPower);
            encoderDriveSmooth("left", 0.5);
            encoderDriveSmoothDist("left", fullSkystoneDist);

            // grab skystone
            encoderDriveSmooth("front", 0.25, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmooth("back", 0.25);

            // depot to building site
            rotate(90, turnPower);
            encoderDriveSmoothDist("front", 1.5*inPerBlock + fullSkystoneDist);
            dropSkystone();
            /*
            encoderDriveSmoothDist("back", 2*inPerBlock + fullSkystoneDist);

            //3rd skystone
            rotate(-90, turnPower);
            encoderDriveSmooth("right", 0.5);
            encoderDriveSmoothDist("left", fullSkystoneDist);

            // grab skystone
            encoderDriveSmooth("front", 0.25, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmoothDist("back", 0.25);

            // depot to building site
            rotate(90, turnPower);
            encoderDriveSmoothDist("front", 1.5*inPerBlock + 2*fullSkystoneDist);
            dropSkystone();
            */
            encoderDriveSmooth("back", 0.625);
            encoderDriveSmooth("left", 0.25);
        }
        stopAllMotors();
    }
}
