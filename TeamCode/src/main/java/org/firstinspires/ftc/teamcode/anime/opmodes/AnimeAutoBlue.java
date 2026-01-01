package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
@Autonomous(name="Anime: Auto Blue", group="Anime")
public class AnimeAutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware here
        
        waitForStart();
        
        // Autonomous code here (30 seconds max)
        // Remember: During AUTO, columns A, B, C constitute the blue side of the FIELD
        
    }
}

