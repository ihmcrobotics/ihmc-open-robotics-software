package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
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
import us.ihmc.sensorProcessing.frames.CommonLeggedReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.util.ArrayList;
import java.util.List;

public class ContactableBodiesFactory<E extends Enum<E> & RobotSegment<E>>
{
   private final RequiredFactoryField<SegmentDependentList<E, ? extends List<Point2D>>> feetContactPoints = new RequiredFactoryField<>("feetContactPoints");
   private final RequiredFactoryField<FullLeggedRobotModel<E>> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<CommonLeggedReferenceFrames<E>> referenceFrames = new RequiredFactoryField<>("referenceFrames");

   private final OptionalFactoryField<SegmentDependentList<E, ? extends Point2D>> toeContactPoints = new OptionalFactoryField<>("toeContactPoints");
   private final OptionalFactoryField<SegmentDependentList<E, ? extends LineSegment2D>> toeContactLines = new OptionalFactoryField<>("toeContactLines");

   private final OptionalFactoryField<ArrayList<String>> additionalContactRigidBodyNames = new OptionalFactoryField<>("additionalContactRigidBodyNames");
   private final OptionalFactoryField<ArrayList<String>> additionalContactNames = new OptionalFactoryField<>("additionalContactNames");
   private final OptionalFactoryField<ArrayList<RigidBodyTransform>> additionalContactTransforms = new OptionalFactoryField<>("additionalContactTransforms");

   public void setFootContactPoints(SegmentDependentList<E, ? extends List<Point2D>> feetContactPoints)
   {
      this.feetContactPoints.set(feetContactPoints);
   }

   public void setFullRobotModel(FullLeggedRobotModel<E> fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setReferenceFrames(CommonLeggedReferenceFrames<E> referenceFrames)
   {
      this.referenceFrames.set(referenceFrames);
   }

   public void setToeContactParameters(SegmentDependentList<E, ? extends Point2D> toeContactPoints, SegmentDependentList<E, ? extends LineSegment2D> toeContactLines)
   {
      this.toeContactPoints.set(toeContactPoints);
      this.toeContactLines.set(toeContactLines);
   }

   public void addAdditionalContactPoint(String bodyName, String contactName, RigidBodyTransform transformFromParentLinkToPoint)
   {
      if (!additionalContactNames.hasValue())
      {
         additionalContactRigidBodyNames.set(new ArrayList<>());
         additionalContactNames.set(new ArrayList<>());
         additionalContactTransforms.set(new ArrayList<>());
      }
      else if (additionalContactRigidBodyNames.get().contains(bodyName))
      {
         // TODO fix this
         throw new RuntimeException("Currently only supporting one additional contact point per rigid body.");
      }

      additionalContactRigidBodyNames.get().add(bodyName);
      additionalContactNames.get().add(contactName);
      additionalContactTransforms.get().add(transformFromParentLinkToPoint);
   }

   public SegmentDependentList<E, ContactablePlaneBody> createFootContactablePlaneBodies()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      FullLeggedRobotModel<E> fullRobotModel = this.fullRobotModel.get();
      CommonLeggedReferenceFrames<E> referenceFrames = this.referenceFrames.get();
      E[] robotSegments = fullRobotModel.getRobotSegments();

      SegmentDependentList<E, ContactablePlaneBody> footContactableBodies = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
      {
         RigidBody foot = fullRobotModel.getFoot(segment);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(segment);
         List<Point2D> contactPointsInSoleFrame = feetContactPoints.get().get(segment);

         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, soleFrame, contactPointsInSoleFrame);
         footContactableBodies.put(segment, footContactableBody);
      }

      feetContactPoints.dispose();
      toeContactPoints.dispose();
      toeContactLines.dispose();

      return footContactableBodies;
   }

   public SegmentDependentList<E, ContactableFoot> createFootContactableFeet()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);
      toeContactLines.get();
      toeContactPoints.get();

      FullLeggedRobotModel<E> fullRobotModel = this.fullRobotModel.get();
      CommonLeggedReferenceFrames<E> referenceFrames = this.referenceFrames.get();
      E[] robotSegments = fullRobotModel.getRobotSegments();

      SegmentDependentList<E, ContactableFoot> footContactableBodies = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
      {
         RigidBody foot = fullRobotModel.getFoot(segment);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(segment);
         List<Point2D> contactPointsInSoleFrame = feetContactPoints.get().get(segment);

         Point2D toeOffContactPoint = toeContactPoints.get().get(segment);
         LineSegment2D toeOffContactLine = toeContactLines.get().get(segment);
         ListOfPointsContactableFoot footContactableBody = new ListOfPointsContactableFoot(foot, soleFrame, contactPointsInSoleFrame, toeOffContactPoint,
               toeOffContactLine);
         footContactableBodies.put(segment, footContactableBody);
      }

      feetContactPoints.dispose();
      toeContactPoints.dispose();
      toeContactLines.dispose();

      return footContactableBodies;
   }

   public List<ContactablePlaneBody> createAdditionalContactPoints()
   {
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      if (!additionalContactNames.hasValue())
         return contactablePlaneBodies;

      FullLeggedRobotModel<E> fullRobotModel = this.fullRobotModel.get();
      RigidBody[] bodies = ScrewTools.computeSubtreeSuccessors(fullRobotModel.getElevator());

      for (int pointIdx = 0; pointIdx < additionalContactRigidBodyNames.get().size(); pointIdx++)
      {
         String bodyName = additionalContactRigidBodyNames.get().get(pointIdx);
         String contactName = additionalContactNames.get().get(pointIdx);
         RigidBodyTransform contactFramePoseInJoint = additionalContactTransforms.get().get(pointIdx);

         RigidBody[] rigidBodies = ScrewTools.findRigidBodiesWithNames(bodies, bodyName);

         if (rigidBodies.length == 0)
            throw new RuntimeException("Did not find body with name " + bodyName);
         if (rigidBodies.length > 1)
            throw new RuntimeException("Found multiple bodies with name " + bodyName);

         RigidBody rigidBody = rigidBodies[0];
         contactablePlaneBodies.add(new SimpleContactPointPlaneBody(contactName, rigidBody, contactFramePoseInJoint));
      }

      return contactablePlaneBodies;
   }

   public void disposeFactory()
   {
      FactoryTools.disposeFactory(this);
   }
}
