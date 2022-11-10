package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class BipedalSupportPlanarRegionCalculator
{
   private static final int LEFT_FOOT_INDEX = 0;
   private static final int RIGHT_FOOT_INDEX = 1;
   private static final int CONVEX_HULL_INDEX = 2;

   private final List<PlanarRegion> supportRegions = new ArrayList<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJointsExcludingHands;
   private final HumanoidReferenceFrames referenceFrames;
   private final SideDependentList<ContactablePlaneBody> contactableFeet;
   private final SideDependentList<List<FramePoint2D>> scaledContactPointList = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

   public BipedalSupportPlanarRegionCalculator(DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      fullRobotModel = robotModel.createFullRobotModel();
      allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      contactableBodiesFactory.setFootContactPoints(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      contactableFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactablePlaneBodies());

      for (int i = 0; i < 3; i++)
      {
         supportRegions.add(new PlanarRegion());
      }
   }

   public void initializeEmptyRegions()
   {
      supportRegions.set(LEFT_FOOT_INDEX, new PlanarRegion());
      supportRegions.set(RIGHT_FOOT_INDEX, new PlanarRegion());
      supportRegions.set(CONVEX_HULL_INDEX, new PlanarRegion());
   }

   public void calculateSupportRegions(double scaleFactor, CapturabilityBasedStatus capturabilityBasedStatus, RobotConfigurationData robotConfigurationData)
   {
      initializeEmptyRegions();

      for (RobotSide robotSide : RobotSide.values)
      {
         scaledContactPointList.get(robotSide).clear();
         for (FramePoint2D contactPoint : contactableFeet.get(robotSide).getContactPoints2d())
         {
            FramePoint2D scaledContactPoint = new FramePoint2D(contactPoint);
            scaledContactPoint.scale(scaleFactor);
            scaledContactPointList.get(robotSide).add(scaledContactPoint);
         }
      }

      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, fullRobotModel.getRootJoint(), allJointsExcludingHands);

      referenceFrames.updateFrames();

      SideDependentList<Boolean> isInSupport = new SideDependentList<Boolean>(!capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty(),
                                                                              !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty());

      if (feetAreInSamePlane(isInSupport))
      {
         ReferenceFrame leftSoleFrame = contactableFeet.get(RobotSide.LEFT).getSoleFrame();

         List<FramePoint2D> allContactPoints = new ArrayList<>();
         allContactPoints.addAll(scaledContactPointList.get(RobotSide.LEFT));
         allContactPoints.addAll(scaledContactPointList.get(RobotSide.RIGHT));
         allContactPoints.forEach(p -> p.changeFrameAndProjectToXYPlane(leftSoleFrame));

         supportRegions.set(LEFT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(RIGHT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(CONVEX_HULL_INDEX, new PlanarRegion(leftSoleFrame.getTransformToWorldFrame(),
                                                                new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(allContactPoints))));
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (isInSupport.get(robotSide))
            {
               ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
               List<FramePoint2D> contactPoints = scaledContactPointList.get(robotSide);
               RigidBodyTransform transformToWorld = contactableFoot.getSoleFrame().getTransformToWorldFrame();
               supportRegions.set(robotSide.ordinal(),
                                  new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPoints))));
            }
            else
            {
               supportRegions.set(robotSide.ordinal(), new PlanarRegion());
            }
         }

         supportRegions.set(CONVEX_HULL_INDEX, new PlanarRegion());
      }
   }

   private boolean feetAreInSamePlane(SideDependentList<Boolean> isInSupport)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isInSupport.get(robotSide))
         {
            return false;
         }
      }
      ReferenceFrame leftSoleFrame = contactableFeet.get(RobotSide.LEFT).getSoleFrame();
      ReferenceFrame rightSoleFrame = contactableFeet.get(RobotSide.RIGHT).getSoleFrame();
      RigidBodyTransform relativeSoleTransform = leftSoleFrame.getTransformToDesiredFrame(rightSoleFrame);
      RotationMatrixReadOnly relativeOrientation = relativeSoleTransform.getRotation();

      double rotationEpsilon = Math.toRadians(3.0);
      double translationEpsilon = 0.02;
      return Math.abs(relativeOrientation.getPitch()) < rotationEpsilon && Math.abs(relativeOrientation.getRoll()) < rotationEpsilon
             && Math.abs(relativeSoleTransform.getTranslationZ()) < translationEpsilon;
   }

   public List<PlanarRegion> getSupportRegions()
   {
      for (int i = 0; i < 3; i++)
      {
         supportRegions.get(i).setRegionId(i);
      }
      return supportRegions;
   }

   public PlanarRegionsList getSupportRegionsAsList()
   {
      ArrayList<PlanarRegion> copiedRegions = new ArrayList<>();
      List<PlanarRegion> supportRegions = getSupportRegions();
      for (PlanarRegion supportRegion : supportRegions)
      {
         copiedRegions.add(supportRegion.copy());
      }
      return new PlanarRegionsList(copiedRegions);
   }
}
