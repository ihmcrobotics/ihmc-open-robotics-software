package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ContactableBodiesFactory
{
   private SideDependentList<? extends List<Point2D>> footContactPoints = null;
   private SideDependentList<? extends Point2D> toeContactPoints = null;

   private final ArrayList<String> rigidBodyNames = new ArrayList<>();
   private final ArrayList<String> contactNames = new ArrayList<>();
   private final ArrayList<RigidBodyTransform> contactPointFrameTransforms = new ArrayList<>();

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

   public void addAdditionalContactPoint(String bodyName, String contactName, RigidBodyTransform transformFromParentLinkToPoint)
   {
      // TODO: fix this.
      if (rigidBodyNames.contains(bodyName))
         throw new RuntimeException("Currently only supporting one additional contact point per rigid body.");

      rigidBodyNames.add(bodyName);
      contactNames.add(contactName);
      contactPointFrameTransforms.add(transformFromParentLinkToPoint);
   }

   public List<ContactablePlaneBody> createAdditionalContactPoints(FullHumanoidRobotModel fullRobotModel)
   {
      ArrayList<ContactablePlaneBody> ret = new ArrayList<>();
      RigidBody[] bodies = ScrewTools.computeSubtreeSuccessors(fullRobotModel.getElevator());

      for (int pointIdx = 0; pointIdx < rigidBodyNames.size(); pointIdx++)
      {
         String bodyName = rigidBodyNames.get(pointIdx);
         String contactName = contactNames.get(pointIdx);
         RigidBodyTransform contactFramePoseInJoint = contactPointFrameTransforms.get(pointIdx);

         RigidBody[] rigidBodies = ScrewTools.findRigidBodiesWithNames(bodies, bodyName);

         if (rigidBodies.length == 0)
            throw new RuntimeException("Did not find body with name " + bodyName);
         if (rigidBodies.length > 1)
            throw new RuntimeException("Found multiple bodies with name " + bodyName);

         RigidBody rigidBody = rigidBodies[0];
         ret.add(new SimpleContactPointPlaneBody(contactName, rigidBody, contactFramePoseInJoint));
      }

      return ret;
   }

}
