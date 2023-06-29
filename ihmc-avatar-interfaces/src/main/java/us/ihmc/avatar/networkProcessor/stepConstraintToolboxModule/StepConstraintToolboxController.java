package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StepConstraintMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.stepAdjustment.SimpleStep;
import us.ihmc.avatar.stepAdjustment.StepConstraintCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintMessageConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StepConstraintToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final StepConstraintCalculator stepConstraintCalculator;

   private final YoDouble time = new YoDouble("time", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final IHMCROS2Publisher<StepConstraintMessage> constraintRegionPublisher;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> configurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   public StepConstraintToolboxController(StatusMessageOutputManager statusOutputManager,
                                          IHMCROS2Publisher<StepConstraintMessage> constraintRegionPublisher,
                                          WalkingControllerParameters walkingControllerParameters,
                                          FullHumanoidRobotModel fullRobotModel,
                                          double gravityZ,
                                          YoRegistry parentRegistry,
                                          YoGraphicsListRegistry graphicsListRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.constraintRegionPublisher = constraintRegionPublisher;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      stepConstraintCalculator = new StepConstraintCalculator(walkingControllerParameters,
                                                              referenceFrames.getSoleZUpFrames(),
                                                              time);

      parentRegistry.addChild(stepConstraintCalculator.getYoVariableRegistry());
      if (graphicsListRegistry != null)
      {
         graphicsListRegistry.registerYoGraphicsLists(stepConstraintCalculator.getYoGraphicsListRegistry().getYoGraphicsLists());
         List<ArtifactList> artifactLists = new ArrayList<>();
         stepConstraintCalculator.getYoGraphicsListRegistry().getRegisteredArtifactLists(artifactLists);
         graphicsListRegistry.registerArtifactLists(artifactLists);
      }

      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);

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
            time.set(Conversions.nanosecondsToSeconds(configurationData.getMonotonicTime()));

            KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(configurationData, fullRobotModel.getRootJoint(), oneDoFJoints);
            referenceFrames.updateFrames();
         }

         CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getAndSet(null);
         if (capturabilityBasedStatus != null)
         {
            stepConstraintCalculator.setRightFootSupportPolygon(capturabilityBasedStatus.getRightFootSupportPolygon3d());
            stepConstraintCalculator.setLeftFootSupportPolygon(capturabilityBasedStatus.getLeftFootSupportPolygon3d());
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
         {
            StepConstraintRegion constraintRegion = stepConstraintCalculator.pollStepConstraintRegion();
            if (constraintRegion != null)
               constraintRegionPublisher.publish(StepConstraintMessageConverter.convertToStepConstraintMessage(constraintRegion));
         }
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
