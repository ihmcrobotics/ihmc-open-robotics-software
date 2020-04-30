package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.PlanarRegionsListHandler;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionTools;
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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleProvider timeProvider;

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private final SideDependentList<SimpleStep> currentSteps = new SideDependentList<>();

   public StepConstraintCalculator(WalkingControllerParameters walkingControllerParameters, FullHumanoidRobotModel fullRobotModel, DoubleProvider timeProvider)
   {
      this.timeProvider = timeProvider;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, null);
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
         currentSteps.put(robotSide, new SimpleStep(statusMessage, timeProvider.getValue()));
      else
         currentSteps.remove(robotSide);
   }

   public void updatePlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      planarRegionsList.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
   }

   public void update()
   {
      referenceFrames.updateFrames();

      updateCaptureRegion();
   }

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final FrameConvexPolygon2D leftFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D rightFootSupportPolygon = new FrameConvexPolygon2D();

   private void updateCaptureRegion()
   {
      capturePoint.set(capturabilityBasedStatus.getCapturePoint2d());
      leftFootSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getLeftFootSupportPolygon2d()));
      rightFootSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getRightFootSupportPolygon2d()));

      if (currentSteps.containsKey(RobotSide.LEFT))
      {
         SimpleStep step = currentSteps.get(RobotSide.LEFT);
         double timeInState = timeProvider.getValue() - step.getStartTime();
         double timeRemaining = step.getSwingDuration() - timeInState;
         captureRegionCalculator.calculateCaptureRegion(RobotSide.LEFT, timeRemaining, capturePoint, capturabilityBasedStatus.getOmega(),
                                                        leftFootSupportPolygon);
      }
      else if (currentSteps.containsKey(RobotSide.RIGHT))
      {
         SimpleStep step = currentSteps.get(RobotSide.RIGHT);
         double timeInState = timeProvider.getValue() - step.getStartTime();
         double timeRemaining = step.getSwingDuration() - timeInState;
         captureRegionCalculator.calculateCaptureRegion(RobotSide.LEFT, timeRemaining, capturePoint, capturabilityBasedStatus.getOmega(),
                                                        rightFootSupportPolygon);
      }
      else
      {
         captureRegionCalculator.hideCaptureRegion();
      }
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
   }
}
