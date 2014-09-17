package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.math.geometry.Transform3d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public abstract class DRCRobotContactPointParameters
{
   public abstract Transform3d getPelvisContactPointTransform();

   public abstract List<Point2d> getPelvisContactPoints();

   public abstract Transform3d getPelvisBackContactPointTransform();

   public abstract List<Point2d> getPelvisBackContactPoints();

   public abstract Transform3d getChestBackContactPointTransform();

   public abstract List<Point2d> getChestBackContactPoints();

   public abstract SideDependentList<Transform3d> getThighContactPointTransforms();

   public abstract SideDependentList<List<Point2d>> getThighContactPoints();

   public abstract List<Pair<String, Vector3d>> getJointNameGroundContactPointMap();

   public abstract SideDependentList<ArrayList<Point2d>> getFootContactPoints();
   
   public abstract ContactableBodiesFactory getContactableBodiesFactory();

   public abstract SideDependentList<Transform3d> getHandContactPointTransforms();

   public abstract SideDependentList<List<Point2d>> getHandContactPoints();
}
