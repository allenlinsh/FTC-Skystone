package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.Prk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BluePrkBridgeBuild", group = "Prk")
public class BluePrkBridgeBuild extends MainAutonomous {
    public String className = getClass().getSimpleName();
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
         * Skystone:                No
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Building Site
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Parking
            encoderDriveSmooth("front", 0.25);
            rotate(90, turnPower);
            armExtend();
            timeDrive("right", minPower, 1000);
            encoderDriveSmooth("left", 1);
            encoderDriveSmooth("front", 1.625);
            timeDrive("left", minPower, 500);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
