package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/** BIOBUZZ background only; does not transform robot poses or trajectory coordinates. */
public final class BiobuzzField {
    private BiobuzzField() { }

    public static void apply(TelemetryPacket packet) {
        Canvas field = packet.field();
        field.clear();
        // The official guide has red on the left. Rotate it to red at the bottom.
        field.setAlpha(0.65);
        field.drawImage("/images/biobuzz-2026-2027.png", 72, 72, 144, 144,
                -Math.PI / 2, 72, 72, true);
        field.setAlpha(1.0);
        field.drawGrid(0, 0, 144, 144, 7, 7);
    }
}
