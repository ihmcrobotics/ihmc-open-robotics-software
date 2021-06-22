package us.ihmc.avatar.networkProcessor.pushRecoveryToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturabilityBasedPlanarRegionDecider;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.IntFunction;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class PushRecoveryToolboxController extends ToolboxController
{
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> configurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> mostRecentPlanarRegions = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsList> planarRegionsToUse = new AtomicReference<>();

   private final IsInContactProvider isInContactProvider = new IsInContactProvider();
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footSupportPolygonsInWorld = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final MultiStepPushRecoveryModule pushRecoveryControlModule;

   private final IHMCRealtimeROS2Publisher<PushRecoveryResultMessage> pushRecoveryResultPublisher;

   private final FramePoint2D capturePoint = new FramePoint2D();
   private double omega;

   public PushRecoveryToolboxController(StatusMessageOutputManager statusOutputManager,
                                        IHMCRealtimeROS2Publisher<PushRecoveryResultMessage> pushRecoveryResultPublisher,
                                        FullHumanoidRobotModel fullRobotModel,
                                        ConvexPolygon2DReadOnly defaultSupportPolygon,
                                        PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                        double gravityZ,
                                        YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.pushRecoveryResultPublisher = pushRecoveryResultPublisher;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);

      pushRecoveryControlModule = new MultiStepPushRecoveryModule(isInContactProvider,
                                                                  supportPolygonInWorld,
                                                                  footSupportPolygonsInWorld,
                                                                  referenceFrames.getSoleZUpFrames(),
                                                                  defaultSupportPolygon,
                                                                  pushRecoveryControllerParameters,
                                                                  parentRegistry,
                                                                  null);
      CapturabilityBasedPlanarRegionDecider<PlanarRegion> planarRegionDecider = new CapturabilityBasedPlanarRegionDecider<>(referenceFrames.getCenterOfMassFrame(),
                                                                                                                            gravityZ,
                                                                                                                            PlanarRegion::new,
                                                                                                                            new YoRegistry("registryToThrowAway"),
                                                                                                                            null);
      IntFunction<PlanarRegionsList> constraintRegionProvider = (index) -> planarRegionsToUse.get();
      pushRecoveryControlModule.setPlanarRegionDecider(planarRegionDecider);
      pushRecoveryControlModule.setConstraintRegionProvider(constraintRegionProvider);
      isDone.set(false);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      return true;
   }

   @Override
   public void updateInternal()
   {
      try
      {
         RobotConfigurationData configurationData = this.configurationData.getAndSet(null);
         if (configurationData != null)
         {
            KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(configurationData, fullRobotModel.getRootJoint(), oneDoFJoints);
            referenceFrames.updateFrames();
         }

         CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getAndSet(null);
         if (capturabilityBasedStatus != null)
         {
            isInContactProvider.setCapturabilityBasedStatus(capturabilityBasedStatus);

            updateSupportPolygons(capturabilityBasedStatus);

            capturePoint.set(capturabilityBasedStatus.getCapturePoint2d());
            omega = capturabilityBasedStatus.getOmega();
         }

         PlanarRegionsListMessage planarRegions = this.mostRecentPlanarRegions.getAndSet(null);
         if (planarRegions != null)
         {
            planarRegionsToUse.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions));
         }

         pushRecoveryControlModule.updateForDoubleSupport(capturePoint, omega);

         PushRecoveryResultMessage message = new PushRecoveryResultMessage();

         if (pushRecoveryControlModule.isRecoveryImpossible())
         {
            message.setIsStepRecoverable(false);
         }
         else
         {
            message.setIsStepRecoverable(true);
            for (int i = 0; i < pushRecoveryControlModule.getNumberOfRecoverySteps(); i++)
            {
               FootstepTiming recoveryTiming = pushRecoveryControlModule.getRecoveryStepTiming(i);
               Footstep step = pushRecoveryControlModule.getRecoveryStep(i);
               FootstepDataMessage stepMessage = message.getRecoverySteps().getFootstepDataList().add();

               stepMessage.setTransferDuration(recoveryTiming.getTransferTime());
               stepMessage.setSwingDuration(recoveryTiming.getSwingTime());
               stepMessage.getLocation().set(step.getFootstepPose().getPosition());
               stepMessage.getOrientation().set(step.getFootstepPose().getOrientation());
            }

            // todo add the step constraint stuff.
         }

         pushRecoveryResultPublisher.publish(message);
      }
      catch (Throwable e)
      {
         e.printStackTrace();

         try
         {
            reportMessage(MessageTools.createControllerCrashNotificationPacket(null, e));
         }
         catch (Exception e1)
         {
            e1.printStackTrace();
         }

         isDone.set(true);
      }
   }

   private void updateSupportPolygons(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      footSupportPolygonsInWorld.get(RobotSide.LEFT).clear();
      footSupportPolygonsInWorld.get(RobotSide.RIGHT).clear();
      supportPolygonInWorld.clear();
      capturabilityBasedStatus.getLeftFootSupportPolygon3d().forEach(point ->
                                                                     {
                                                                        footSupportPolygonsInWorld.get(RobotSide.LEFT).addVertex(point);
                                                                        supportPolygonInWorld.addVertex(point);
                                                                     });
      capturabilityBasedStatus.getRightFootSupportPolygon3d().forEach(point ->
                                                                      {
                                                                         footSupportPolygonsInWorld.get(RobotSide.RIGHT).addVertex(point);
                                                                         supportPolygonInWorld.addVertex(point);
                                                                      });
      footSupportPolygonsInWorld.get(RobotSide.LEFT).update();
      footSupportPolygonsInWorld.get(RobotSide.RIGHT).update();
      supportPolygonInWorld.update();
   }

   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   private static class IsInContactProvider implements Function<RobotSide, Boolean>
   {
      private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      public void setCapturabilityBasedStatus(CapturabilityBasedStatus status)
      {
         this.capturabilityBasedStatus.set(status);
      }

      public Boolean apply(RobotSide robotSide)
      {
         if (robotSide == RobotSide.LEFT)
            return capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty();
         else
            return capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty();
      }
   }
}
