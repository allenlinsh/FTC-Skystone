package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.Park;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BluePrkBridgeDep extends MainAutonomous {
    private String className = this.getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences();
        print("Alliance", teamColor);
        print("Foundation", doFoundation);
        print("Skystone", doSkystone);
        print("Parking Position", parking);
        print("Starting Position", starting);

        // Initialize hardware
        getHardwareMap();
        initCheck();
        print("Status", "Initialized");
        getPreferences();
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
         * Start Position:          Depot
         */
        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            encoderDrive("front", minPower, 1);
            encoderDrive("left", minPower, 1.625);
        }

        resetMotor();
        visionTargets.deactivate();
    }
}
