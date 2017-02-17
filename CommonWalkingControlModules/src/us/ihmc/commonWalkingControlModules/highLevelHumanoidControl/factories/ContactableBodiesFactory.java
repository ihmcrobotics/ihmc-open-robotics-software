package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   private SideDependentList<List<Point2D>> handContactPoints = null;
   private SideDependentList<RigidBodyTransform> handContactPointTransforms = null;

   private SideDependentList<? extends List<Point2D>> footContactPoints = null;
   private SideDependentList<? extends Point2D> toeContactPoints = null;

   public void addHandContactParameters(SideDependentList<String> namesOfJointsBeforeHands, SideDependentList<List<Point2D>> handContactPoints,
         SideDependentList<RigidBodyTransform> handContactPointTransforms)
   {
      this.namesOfJointsBeforeHands = namesOfJointsBeforeHands;
      this.handContactPoints = handContactPoints;
      this.handContactPointTransforms = handContactPointTransforms;
   }

   public void addFootContactParameters(SideDependentList<? extends List<Point2D>> footContactPoints, SideDependentList<? extends Point2D> toeContactPoints)
   {
      this.footContactPoints = footContactPoints;
      this.toeContactPoints = toeContactPoints;
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

   private final static ListOfPointsContactablePlaneBody createListOfPointsContactablePlaneBody(String name, RigidBody body,
         RigidBodyTransform contactPointsTransform, List<Point2D> contactPoints)
   {
      ReferenceFrame parentFrame = body.getParentJoint().getFrameAfterJoint();
      ReferenceFrame contactPointsFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, parentFrame, contactPointsTransform);
      ListOfPointsContactablePlaneBody ret = new ListOfPointsContactablePlaneBody(body, contactPointsFrame, contactPoints);
      return ret;
   }
}
