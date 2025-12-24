package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
@Autonomous(name="Anime: Auto Red", group="Anime")
public class AnimeAutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware here
        
        waitForStart();
        
        // Autonomous code here (30 seconds max)
        // Remember: During AUTO, columns D, E, F constitute the red side of the FIELD
        
    }
}

