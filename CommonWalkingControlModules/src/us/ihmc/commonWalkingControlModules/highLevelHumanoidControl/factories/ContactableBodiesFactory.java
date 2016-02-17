package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ContactableBodiesFactory
{
   private SideDependentList<String> namesOfJointsBeforeHands = null;
   private SideDependentList<List<Point2d>> handContactPoints = null;
   private SideDependentList<RigidBodyTransform> handContactPointTransforms = null;

   private SideDependentList<? extends List<Point2d>> footContactPoints = null;

   public void addHandContactParameters(SideDependentList<String> namesOfJointsBeforeHands, SideDependentList<List<Point2d>> handContactPoints,
         SideDependentList<RigidBodyTransform> handContactPointTransforms)
   {
      this.namesOfJointsBeforeHands = namesOfJointsBeforeHands;
      this.handContactPoints = handContactPoints;
      this.handContactPointTransforms = handContactPointTransforms;
   }

   public void addFootContactParameters(SideDependentList<? extends List<Point2d>> footContactPoints)
   {
      this.footContactPoints = footContactPoints;
   }

   public SideDependentList<ContactablePlaneBody> createHandContactableBodies(RigidBody rootBody)
   {
      if (namesOfJointsBeforeHands == null)
         return null;

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootBody);

      SideDependentList<ContactablePlaneBody> handContactableBodies = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         InverseDynamicsJoint[] jointBeforeHandArray = ScrewTools.findJointsWithNames(allJoints, namesOfJointsBeforeHands.get(robotSide));
         if (jointBeforeHandArray.length != 1)
            throw new RuntimeException("Incorrect number of joints before hand found: " + jointBeforeHandArray.length);

         RigidBody hand = jointBeforeHandArray[0].getSuccessor();
         String name = robotSide.getCamelCaseNameForStartOfExpression() + "HandContact";
         ListOfPointsContactablePlaneBody handContactableBody = createListOfPointsContactablePlaneBody(name, hand, handContactPointTransforms.get(robotSide),
               handContactPoints.get(robotSide));
         handContactableBodies.put(robotSide, handContactableBody);
      }
      return handContactableBodies;
   }

   public SideDependentList<ContactablePlaneBody> createFootContactableBodies(FullHumanoidRobotModel fullRobotModel,
         CommonHumanoidReferenceFrames referenceFrames)
   {
      if (footContactPoints == null)
         return null;

      SideDependentList<ContactablePlaneBody> footContactableBodies = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, referenceFrames.getSoleFrame(robotSide),
               footContactPoints.get(robotSide));
         footContactableBodies.put(robotSide, footContactableBody);
      }
      return footContactableBodies;
   }

   private final static ListOfPointsContactablePlaneBody createListOfPointsContactablePlaneBody(String name, RigidBody body,
         RigidBodyTransform contactPointsTransform, List<Point2d> contactPoints)
   {
      ReferenceFrame parentFrame = body.getParentJoint().getFrameAfterJoint();
      ReferenceFrame contactPointsFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, parentFrame, contactPointsTransform);
      ListOfPointsContactablePlaneBody ret = new ListOfPointsContactablePlaneBody(body, contactPointsFrame, contactPoints);
      return ret;
   }
}
