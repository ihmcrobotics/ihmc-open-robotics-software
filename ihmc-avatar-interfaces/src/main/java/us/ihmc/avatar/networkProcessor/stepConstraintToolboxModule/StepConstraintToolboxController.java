package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class StepConstraintToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final StepConstraintCalculator stepConstraintCalculator;

   private final YoDouble time = new YoDouble("time", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final IHMCRealtimeROS2Publisher<PlanarRegionMessage> planarRegionConstraintPublisher;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> configurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   public StepConstraintToolboxController(StatusMessageOutputManager statusOutputManager,
                                          IHMCRealtimeROS2Publisher<PlanarRegionMessage> planarRegionConstraintPublisher,
                                          WalkingControllerParameters walkingControllerParameters,
                                          FullHumanoidRobotModel fullRobotModel,
                                          double gravityZ,
                                          YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.planarRegionConstraintPublisher = planarRegionConstraintPublisher;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      stepConstraintCalculator = new StepConstraintCalculator(walkingControllerParameters,
                                                              referenceFrames.getSoleZUpFrames(),
                                                              referenceFrames.getCenterOfMassFrame(),
                                                              time,
                                                              gravityZ);

      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);

      isDone.set(false);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      return true;
   }

   private long initialTimestamp = -1L;

   @Override
   public void updateInternal()
   {
      try
      {
         if (initialTimestamp == -1L)
            initialTimestamp = System.nanoTime();
         time.set(Conversions.nanosecondsToSeconds(System.nanoTime() - initialTimestamp));

         RobotConfigurationData configurationData = this.configurationData.getAndSet(null);
         if (configurationData != null)
         {
            KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(configurationData, fullRobotModel.getRootJoint(), oneDoFJoints);
            referenceFrames.updateFrames();
         }

         CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getAndSet(null);
         if (capturabilityBasedStatus != null)
         {
            stepConstraintCalculator.setRightFootSupportPolygon(capturabilityBasedStatus.getRightFootSupportPolygon2d());
            stepConstraintCalculator.setLeftFootSupportPolygon(capturabilityBasedStatus.getLeftFootSupportPolygon2d());
            stepConstraintCalculator.setOmega(capturabilityBasedStatus.getOmega());
            stepConstraintCalculator.setCapturePoint(capturabilityBasedStatus.getCapturePoint2d());
         }

         FootstepStatusMessage footstepStatusMessage = this.footstepStatusMessage.getAndSet(null);
         if (footstepStatusMessage != null)
         {
            if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.STARTED)
            {
               SimpleStep simpleStep = new SimpleStep(footstepStatusMessage, time.getDoubleValue());
               stepConstraintCalculator.setCurrentStep(simpleStep);
            }
            else
            {
               stepConstraintCalculator.reset();
            }
         }

         PlanarRegionsListMessage planarRegions = this.planarRegions.getAndSet(null);
         if (planarRegions != null)
         {
            stepConstraintCalculator.setPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions));
         }

         stepConstraintCalculator.update();

         if (stepConstraintCalculator.constraintRegionChanged())
            planarRegionConstraintPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionMessage(stepConstraintCalculator.getConstraintRegion()));
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

   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      stepConstraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(switchAutomatically);
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      this.configurationData.set(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      this.capturabilityBasedStatus.set(newStatus);
   }

   public void updateFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   public void updatePlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.planarRegions.set(planarRegionsListMessage);
   }

   public double getTime()
   {
      return time.getDoubleValue();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
