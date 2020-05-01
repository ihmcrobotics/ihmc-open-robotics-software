package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class StepConstraintCalculator
{
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final HumanoidReferenceFrames referenceFrames;
   private final ICPControlPlane icpControlPlane;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleProvider timeProvider;
   private final CapturabilityPlanarRegionDecider planarRegionDecider;

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private final SideDependentList<SimpleStep> currentSteps = new SideDependentList<>();

   public StepConstraintCalculator(WalkingControllerParameters walkingControllerParameters, FullHumanoidRobotModel fullRobotModel, DoubleProvider timeProvider,
                                   double gravityZ)
   {
      this.timeProvider = timeProvider;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, null);
      this.icpControlPlane = new ICPControlPlane(referenceFrames.getCenterOfMassFrame(), gravityZ, registry);
      this.planarRegionDecider = new CapturabilityPlanarRegionDecider(captureRegionCalculator, icpControlPlane, registry, null);
      this.rootJoint = fullRobotModel.getRootJoint();
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
   }

   // FIXME make this thread safe
   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      this.capturabilityBasedStatus.set(capturabilityBasedStatus);
   }

   // FIXME make this thread safe
   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(newConfigurationData, rootJoint, oneDoFJoints);
   }

   // FIXME make this thread safe
   public void updateFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide robotSide = RobotSide.fromByte(statusMessage.getRobotSide());
      if (FootstepStatus.fromByte(statusMessage.getFootstepStatus()) == FootstepStatus.STARTED)
      {
         currentSteps.put(robotSide, new SimpleStep(statusMessage, timeProvider.getValue()));
         planarRegionDecider.reset();
      }
      else
      {
         currentSteps.remove(robotSide);
      }
   }

   public void updatePlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      planarRegionsList.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      planarRegionDecider.setPlanarRegions(planarRegionsList.get().getPlanarRegionsAsList());
   }

   public void update()
   {
      referenceFrames.updateFrames();
      icpControlPlane.setOmega0(capturabilityBasedStatus.getOmega());

      SimpleStep simpleStep;
      if (currentSteps.containsKey(RobotSide.LEFT))
      {
         simpleStep = currentSteps.get(RobotSide.LEFT);
      }
      else if (currentSteps.containsKey(RobotSide.RIGHT))
      {
         simpleStep = currentSteps.get(RobotSide.RIGHT);
      }
      else
      {
         simpleStep = null;
      }

      updateCaptureRegion(simpleStep);

      if (simpleStep != null)
         planarRegionDecider.updatePlanarRegionConstraintForStep(simpleStep.getStepPose());
   }

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final FrameConvexPolygon2D leftFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D rightFootSupportPolygon = new FrameConvexPolygon2D();

   private void updateCaptureRegion(SimpleStep simpleStep)
   {
      capturePoint.set(capturabilityBasedStatus.getCapturePoint2d());
      leftFootSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getLeftFootSupportPolygon2d()));
      rightFootSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getRightFootSupportPolygon2d()));
      leftFootSupportPolygon.update();
      rightFootSupportPolygon.update();

      if (simpleStep == null)
      {
         captureRegionCalculator.hideCaptureRegion();
         return;
      }

      double timeInState = timeProvider.getValue() - simpleStep.getStartTime();
      double timeRemaining = simpleStep.getSwingDuration() - timeInState;
      RobotSide swingSide = simpleStep.getSwingSide();

      if (swingSide == RobotSide.LEFT)
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, capturabilityBasedStatus.getOmega(),
                                                        rightFootSupportPolygon);
      }
      else
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, capturabilityBasedStatus.getOmega(),
                                                        leftFootSupportPolygon);
      }
   }

   public boolean constraintRegionChanged()
   {
      return planarRegionDecider.constraintRegionChanged();
   }

   public PlanarRegion getConstraintRegion()
   {
      return planarRegionDecider.getConstraintRegion();
   }

   private class SimpleStep
   {
      private final RobotSide swingSide;
      private final double swingDuration;
      private final double startTime;
      private final FramePose3D stepPose = new FramePose3D();

      public SimpleStep(FootstepStatusMessage statusMessage, double startTime)
      {
         this.swingSide = RobotSide.fromByte(statusMessage.getRobotSide());
         this.swingDuration = statusMessage.getSwingDuration();
         this.startTime = startTime;
         stepPose.getPosition().set(statusMessage.getDesiredFootPositionInWorld());
         stepPose.getOrientation().set(statusMessage.getDesiredFootOrientationInWorld());
      }

      public double getSwingDuration()
      {
         return swingDuration;
      }

      public double getStartTime()
      {
         return startTime;
      }

      public RobotSide getSwingSide()
      {
         return swingSide;
      }

      public FramePose3DReadOnly getStepPose()
      {
         return stepPose;
      }
   }
}
