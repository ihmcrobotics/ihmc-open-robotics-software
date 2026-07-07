package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullLeggedRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;
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

   private final OptionalFactoryField<SegmentDependentList<E, String>> secondaryFootContactBodyNames = new OptionalFactoryField<>("secondaryFootContactBodyNames");
   private final OptionalFactoryField<SegmentDependentList<E, RigidBodyTransform>> secondaryFootContactTransforms = new OptionalFactoryField<>("secondaryFootContactTransforms");
   private final OptionalFactoryField<SegmentDependentList<E, Point2D>> secondaryFootContactPointsInSoleFrame = new OptionalFactoryField<>("secondaryFootContactPointsInSoleFrame");

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
         RigidBodyBasics foot = fullRobotModel.getFoot(segment);
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
         RigidBodyBasics foot = fullRobotModel.getFoot(segment);
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

   /**
    * Declares an optional secondary foot contact for the given segment: a single contact point on a rigid body distinct
    * from the main foot (e.g. the second plate of a split foot). {@code transformFromParentJointToContact} is the pose
    * of the contact frame in the body's parent-joint frame; {@code contactPointInSoleFrame} is the same point expressed
    * in the (primary) sole frame, used by whole-foot geometric consumers (default foot polygon, CoP planning).
    */
   public void setSecondaryFootContactPoint(E segment, String bodyName, RigidBodyTransform transformFromParentJointToContact,
                                            Point2D contactPointInSoleFrame)
   {
      if (!secondaryFootContactBodyNames.hasValue())
      {
         Class<E> clazz = segment.getClassType();
         secondaryFootContactBodyNames.set(new SegmentDependentList<>(clazz));
         secondaryFootContactTransforms.set(new SegmentDependentList<>(clazz));
         secondaryFootContactPointsInSoleFrame.set(new SegmentDependentList<>(clazz));
      }

      secondaryFootContactBodyNames.get().put(segment, bodyName);
      secondaryFootContactTransforms.get().put(segment, transformFromParentJointToContact);
      secondaryFootContactPointsInSoleFrame.get().put(segment, contactPointInSoleFrame);
   }

   /** Sole-frame points of the declared secondary foot contacts, or {@code null} when none were declared. */
   public SegmentDependentList<E, Point2D> getSecondaryFootContactPointsInSoleFrame()
   {
      return secondaryFootContactPointsInSoleFrame.hasValue() ? secondaryFootContactPointsInSoleFrame.get() : null;
   }

   /**
    * Creates the secondary foot contactable bodies declared via
    * {@link #setSecondaryFootContactPoint(Enum, String, RigidBodyTransform)}. Returns {@code null} when none were
    * declared; sides without a secondary contact are simply absent from the returned list.
    */
   public SegmentDependentList<E, ContactablePlaneBody> createSecondaryFootContacts()
   {
      if (!secondaryFootContactBodyNames.hasValue())
         return null;

      FullLeggedRobotModel<E> fullRobotModel = this.fullRobotModel.get();
      RigidBodyBasics[] bodies = fullRobotModel.getElevator().subtreeArray();
      E[] robotSegments = fullRobotModel.getRobotSegments();

      SegmentDependentList<E, ContactablePlaneBody> secondaryFootContacts = new SegmentDependentList<>(robotSegments[0].getClassType());

      for (E segment : robotSegments)
      {
         String bodyName = secondaryFootContactBodyNames.get().get(segment);
         if (bodyName == null)
            continue;

         RigidBodyBasics[] rigidBodies = ScrewTools.findRigidBodiesWithNames(bodies, bodyName);
         if (rigidBodies.length != 1)
            throw new RuntimeException("Expected exactly one body with name " + bodyName + ", found " + rigidBodies.length);

         String contactName = segment.toString().toLowerCase() + "SecondaryFootContact";
         secondaryFootContacts.put(segment,
                                   new SimpleContactPointPlaneBody(contactName, rigidBodies[0], secondaryFootContactTransforms.get().get(segment)));
      }

      return secondaryFootContacts;
   }

   public List<ContactablePlaneBody> createAdditionalContactPoints()
   {
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      if (!additionalContactNames.hasValue())
         return contactablePlaneBodies;

      FullLeggedRobotModel<E> fullRobotModel = this.fullRobotModel.get();
      RigidBodyBasics[] bodies = fullRobotModel.getElevator().subtreeArray();

      for (int pointIdx = 0; pointIdx < additionalContactRigidBodyNames.get().size(); pointIdx++)
      {
         String bodyName = additionalContactRigidBodyNames.get().get(pointIdx);
         String contactName = additionalContactNames.get().get(pointIdx);
         RigidBodyTransform contactFramePoseInJoint = additionalContactTransforms.get().get(pointIdx);

         RigidBodyBasics[] rigidBodies = ScrewTools.findRigidBodiesWithNames(bodies, bodyName);

         if (rigidBodies.length == 0)
            throw new RuntimeException("Did not find body with name " + bodyName);
         if (rigidBodies.length > 1)
            throw new RuntimeException("Found multiple bodies with name " + bodyName);

         RigidBodyBasics rigidBody = rigidBodies[0];
         contactablePlaneBodies.add(new SimpleContactPointPlaneBody(contactName, rigidBody, contactFramePoseInJoint));
      }

      return contactablePlaneBodies;
   }

   public void disposeFactory()
   {
      FactoryTools.disposeFactory(this);
   }
}
