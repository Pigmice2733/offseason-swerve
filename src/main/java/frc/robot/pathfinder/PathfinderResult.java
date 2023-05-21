package frc.robot.pathfinder;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DrivetrainConfig;

public class PathfinderResult {
    public final boolean pathFound;
    public final ArrayList<Translation2d> simplifiedPath;
    public final ArrayList<Node> path;

    public PathfinderResult(boolean pathFound, ArrayList<Translation2d> simplifiedPath, ArrayList<Node> path) {
        this.pathFound = pathFound;
        this.simplifiedPath = simplifiedPath;
        this.path = path;
    }

    public PathPlannerTrajectory getAsTrajectory() {
        if (!pathFound) return null;
        if (simplifiedPath.size() < 2) return null;

        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        for (int i = 0; i < simplifiedPath.size()-1; i++) {
            Translation2d current = simplifiedPath.get(i);
            Translation2d next = simplifiedPath.get(i+1);
            Rotation2d angleToNext = Rotation2d.fromRadians(Math.atan2(
                next.getY() - current.getY(),
                next.getX() - current.getX()));
        
            points.add(new PathPoint(current, angleToNext));
        }

        points.add(new PathPoint(
            simplifiedPath.get(simplifiedPath.size()-1), 
            points.get(points.size()-1).heading));

        return PathPlanner.generatePath(DrivetrainConfig.PATH_CONSTRAINTS, points);
    }
}
