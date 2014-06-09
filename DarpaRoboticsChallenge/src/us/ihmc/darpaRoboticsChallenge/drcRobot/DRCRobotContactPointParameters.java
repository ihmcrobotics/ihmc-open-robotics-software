package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public abstract class DRCRobotContactPointParameters
{
   public abstract Transform3D getPelvisContactPointTransform();

   public abstract List<Point2d> getPelvisContactPoints();

   public abstract Transform3D getPelvisBackContactPointTransform();

   public abstract List<Point2d> getPelvisBackContactPoints();

   public abstract Transform3D getChestBackContactPointTransform();

   public abstract List<Point2d> getChestBackContactPoints();

   public abstract SideDependentList<Transform3D> getThighContactPointTransforms();

   public abstract SideDependentList<List<Point2d>> getThighContactPoints();

   public abstract List<Pair<String, Vector3d>> getFootContactPoints(RobotSide robotSide);

   public abstract List<Pair<String, Vector3d>> getThighContactPoints(RobotSide robotSide);

   public abstract List<Pair<String, Vector3d>> getHandContactPoints(RobotSide robotSide);

   public abstract List<Pair<String, Vector3d>> getJointNameGroundContactPointMap();

   public abstract SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController();
}
