package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;

public final class LoadPath {
    public static PathPlannerPath human_player_pickup;
    public static PathPlannerPath coralA;
    public static PathPlannerPath coralB;
    public static PathPlannerPath coralC;
    public static PathPlannerPath coralD;
    public static PathPlannerPath coralE;
    public static PathPlannerPath coralF;
    public static PathPlannerPath coralG;
    public static PathPlannerPath coralH;
    public static PathPlannerPath coralI;
    public static PathPlannerPath coralJ;
    public static PathPlannerPath coralK;
    public static PathPlannerPath coralL;


    public static void init() {
        try {
        human_player_pickup = PathPlannerPath.fromPathFile("Auto3S4");
        coralA = PathPlannerPath.fromPathFile("Coral A");
        coralB = PathPlannerPath.fromPathFile("Coral B");
        coralC = PathPlannerPath.fromPathFile("Coral C");
        coralD = PathPlannerPath.fromPathFile("Coral D");
        coralE = PathPlannerPath.fromPathFile("Coral E");
        coralF = PathPlannerPath.fromPathFile("Coral F");
        coralG = PathPlannerPath.fromPathFile("Coral G");
        coralH = PathPlannerPath.fromPathFile("Coral H");
        coralI = PathPlannerPath.fromPathFile("Coral I");
        coralJ = PathPlannerPath.fromPathFile("Coral J");
        coralK = PathPlannerPath.fromPathFile("Coral K");
        coralL = PathPlannerPath.fromPathFile("Coral L");


        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}