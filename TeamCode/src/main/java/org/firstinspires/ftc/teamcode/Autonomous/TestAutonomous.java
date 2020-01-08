package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutonomous", group = "Test")
public class TestAutonomous extends MainAutonomous {
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

        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Parking
            //timeDrive("front",minPower,1000);
            //timeTurn("clockwise", minTurnPower, 500);
            gyroTurn(90, minTurnPower);
            //encoderDrive("right", minPower, 0.25);
            //armExtend();
            //gripRelease(300);
            //encoderDrive("back", minPower, 1.625);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
