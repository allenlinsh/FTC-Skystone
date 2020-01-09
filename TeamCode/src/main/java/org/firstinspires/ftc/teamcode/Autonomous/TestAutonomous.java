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
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Parking
            timeDrive("front",minPower,1000);
            timeDrive("back",minPower,1000);
            timeTurn("clockwise", minTurnPower, 500);
            timeTurn("counter-clockwise", minTurnPower, 500);
            encoderDrive("front", minPower, 1);
            encoderDrive("back", minPower, 1);
            gyroTurn(90,minTurnPower);
            gyroTurn(-90,minTurnPower);
            armExtend();
            armCollapse();
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
