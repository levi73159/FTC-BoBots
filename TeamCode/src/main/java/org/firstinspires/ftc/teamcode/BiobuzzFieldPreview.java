package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** View the season map without initializing or moving any hardware. */
@TeleOp(name = "BIOBUZZ Field Map", group = "quickstart")
public final class BiobuzzFieldPreview extends LinearOpMode {
    @Override
    public void runOpMode() {
        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket(false);
            BiobuzzField.apply(packet);
            packet.put("Field", "2026-2027 BIOBUZZ");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addLine("Open Dashboard Field View to see the BIOBUZZ map.");
            telemetry.update();
            sleep(100);
        }
    }
}
