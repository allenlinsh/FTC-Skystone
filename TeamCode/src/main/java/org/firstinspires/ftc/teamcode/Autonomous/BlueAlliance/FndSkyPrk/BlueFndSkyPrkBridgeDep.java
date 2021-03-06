package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndSkyPrk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueFndSkyPrkBridgeDep", group = "FndSkyPrk")
public class BlueFndSkyPrkBridgeDep extends MainAutonomous {
    public String className = getClass().getSimpleName();
    public int numDelivered = 1;
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
        update();

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

        waitForStart();
        print("Status", "Running");
        update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                Yes
         * Foundation:              Yes
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
            armRaise(armDuration*4);
            encoderDriveSmooth("back", 0.25);

            // depot to building site
            rotate(-90, turnPower);
            encoderDriveSmooth("front", 2.5);
            rotate(90, turnPower);

            dropSkystone();
            armCollapse();

            rotate(90, turnPower);
            rotate(90, turnPower);

            encoderDriveSmooth("right", 0.25);
            hookOn();

            encoderDriveSmooth("front", 1.25 + foundationMovementError);
            curve(-90, turnPower);
            encoderDriveSmooth("back", 0.5);
            hookOff();
            // Parking
            timeDrive("front", minPower, 100);
            encoderDriveSmoothDist("left", centerPlacement + 0.25*inPerBlock);
            armExtend();
            encoderDriveSmooth("front", 1.625);
            encoderDriveSmooth("left", 0.25, minPower);

            stopAllMotors();

        }
    }
}