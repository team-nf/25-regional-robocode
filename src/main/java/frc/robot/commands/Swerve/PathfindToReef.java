package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LoadPath;

public class PathfindToReef {
    private Command pathfindCommand;

    public static class id {
        private final int id;
        private PathConstraints constraints = 
        new PathConstraints(null, null, null, null);
        private PathPlannerPath correctionPath = null;
        private boolean correctionBranch = false;
        private boolean leftBranch = false;
        private boolean rightBranch = false;
        public id(int id) {
            this.id = id;
        }
        public id withConstraints(PathConstraints constraints) {
            this.constraints = constraints;
            return this;
        }
        public id withCorrectionPath(PathPlannerPath correctionPath) {
            this.correctionPath = correctionPath;
            return this;
        }
        public id withCorrectionPath() {
            if (correctionPath == null) {
                    switch (id) {
                        case 17:
                        if (leftBranch) {this.correctionPath = LoadPath.coralC;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralD;}
                        case 18:
                        if (leftBranch) {this.correctionPath = LoadPath.coralA;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralB;}
                        case 19:
                        if (leftBranch) {this.correctionPath = LoadPath.coralK;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralL;}
                        case 20:
                        if (leftBranch) {this.correctionPath = LoadPath.coralI;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralJ;}
                        case 21:
                        if (leftBranch) {this.correctionPath = LoadPath.coralG;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralH;}
                        case 22:
                        if (leftBranch) {this.correctionPath = LoadPath.coralE;}
                        if (rightBranch) {this.correctionPath = LoadPath.coralF;}
                    
                        default:
                        System.out.println("CRITICAL ERROR: Issue in pathfinding.");
                    }
                }
            this.correctionBranch = false;
            return this;
        }
        public id toLeftBranch() {
            this.correctionBranch = !this.correctionBranch;
            this.leftBranch = true;
            return this;
        }
        public id toRightBranch() {
            this.correctionBranch = !this.correctionBranch;
            this.rightBranch = true;
            return this;
        }
        public PathfindToReef getPath() {return new PathfindToReef(this);}
    }
    private PathfindToReef(id id) {
        if (id.correctionPath != null) {
        this.pathfindCommand = AutoBuilder.pathfindToPose(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(id.id).get().toPose2d(), 
            id.constraints)
            .andThen(AutoBuilder.pathfindThenFollowPath(id.correctionPath, id.constraints));
        } else if (id.correctionBranch == true) {
        this.pathfindCommand = AutoBuilder.pathfindToPose(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(id.id).get().toPose2d(), 
            id.constraints);
            //.andThen(AutoBuilder.pathfindThenFollowPath(id.correctionPath, id.constraints));
            // yazmam lazım
        } else {
        this.pathfindCommand = AutoBuilder.pathfindToPose(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(id.id).get().toPose2d(), 
            id.constraints);
        }
    }
    protected PathfindToReef() {}
    public Command command() {
        return this.pathfindCommand;
    }
}
