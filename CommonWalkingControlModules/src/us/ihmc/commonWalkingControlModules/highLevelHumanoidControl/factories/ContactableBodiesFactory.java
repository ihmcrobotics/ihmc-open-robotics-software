package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ContactableBodiesFactory
{
   private SideDependentList<? extends List<Point2D>> footContactPoints = null;
   private SideDependentList<? extends Point2D> toeContactPoints = null;

   public void addFootContactParameters(SideDependentList<? extends List<Point2D>> footContactPoints, SideDependentList<? extends Point2D> toeContactPoints)
   {
      this.footContactPoints = footContactPoints;
      this.toeContactPoints = toeContactPoints;
   }

   public SideDependentList<ContactableFoot> createFootContactableBodies(FullHumanoidRobotModel fullRobotModel,
         CommonHumanoidReferenceFrames referenceFrames)
   {
      if (footContactPoints == null)
         return null;

      SideDependentList<ContactableFoot> footContactableBodies = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         List<Point2D> contactPointsInSoleFrame = footContactPoints.get(robotSide);
         Point2D toeOffContactPoint = toeContactPoints.get(robotSide);
         ListOfPointsContactableFoot footContactableBody = new ListOfPointsContactableFoot(foot, soleFrame, contactPointsInSoleFrame, toeOffContactPoint);
         footContactableBodies.put(robotSide, footContactableBody);
      }
      return footContactableBodies;
   }

   public Map<RigidBody, ContactablePlaneBody> createFootContactableBodiesMap(FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames)
   {
      if (footContactPoints == null)
         return null;

      Map<RigidBody, ContactablePlaneBody> footContactableBodies = new LinkedHashMap<RigidBody, ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, referenceFrames.getSoleFrame(robotSide),
               footContactPoints.get(robotSide));
         footContactableBodies.put(footContactableBody.getRigidBody(), footContactableBody);
      }
      return footContactableBodies;
   }

}
