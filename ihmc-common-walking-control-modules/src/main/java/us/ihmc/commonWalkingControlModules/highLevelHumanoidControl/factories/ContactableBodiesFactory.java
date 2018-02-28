package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullLeggedRobotModel;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.CommonLeggedReferenceFrames;

import java.util.ArrayList;
import java.util.List;

public class ContactableBodiesFactory<E extends Enum<E> & RobotSegment<E>>
{
   private SegmentDependentList<E, ? extends List<Point2D>> footContactPoints = null;
   private SegmentDependentList<E, ? extends Point2D> toeContactPoints = null;
   private SegmentDependentList<E, ? extends LineSegment2D> toeContactLines = null;

   private final ArrayList<String> rigidBodyNames = new ArrayList<>();
   private final ArrayList<String> contactNames = new ArrayList<>();
   private final ArrayList<RigidBodyTransform> contactPointFrameTransforms = new ArrayList<>();

   private final E[] robotSegments;

   public ContactableBodiesFactory(E[] robotSegments)
   {
      this.robotSegments = robotSegments;
   }

   public void addFootContactParameters(SegmentDependentList<E, ? extends List<Point2D>> footContactPoints, SegmentDependentList<E, ? extends Point2D> toeContactPoints,
         SegmentDependentList<E, ? extends LineSegment2D> toeContactLines)
   {
      this.footContactPoints = footContactPoints;
      this.toeContactPoints = toeContactPoints;
      this.toeContactLines = toeContactLines;
   }

   public SegmentDependentList<E, ContactableFoot> createFootContactableBodies(FullLeggedRobotModel<E> fullRobotModel,
         CommonLeggedReferenceFrames<E> referenceFrames)
   {
      if (footContactPoints == null)
         return null;

      SegmentDependentList<E, ContactableFoot> footContactableBodies = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
      {
         RigidBody foot = fullRobotModel.getFoot(segment);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(segment);
         List<Point2D> contactPointsInSoleFrame = footContactPoints.get(segment);
         Point2D toeOffContactPoint = toeContactPoints.get(segment);
         LineSegment2D toeOffContactLine = toeContactLines.get(segment);
         ListOfPointsContactableFoot footContactableBody = new ListOfPointsContactableFoot(foot, soleFrame, contactPointsInSoleFrame, toeOffContactPoint,
               toeOffContactLine);
         footContactableBodies.put(segment, footContactableBody);
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

   public List<ContactablePlaneBody> createAdditionalContactPoints(FullLeggedRobotModel<E> fullRobotModel)
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
