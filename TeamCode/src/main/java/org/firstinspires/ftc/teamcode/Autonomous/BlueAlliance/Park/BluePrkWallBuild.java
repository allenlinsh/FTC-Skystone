package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.Park;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BluePrkWallBuild extends MainAutonomous {
    private String className = this.getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences();

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
         * Parking Position:        Wall
         * Start Position:          Building Site
         */
        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            encoderDrive("right", minPower, 1.625);
        }

        resetMotor();
        visionTargets.deactivate();
    }
}
