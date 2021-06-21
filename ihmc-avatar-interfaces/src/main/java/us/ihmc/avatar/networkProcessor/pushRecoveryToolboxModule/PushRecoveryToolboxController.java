package us.ihmc.avatar.networkProcessor.pushRecoveryToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryController;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class PushRecoveryToolboxController extends ToolboxController
{
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> configurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();

   private final IsInContactProvider isInContactProvider = new IsInContactProvider();
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footSupportPolygonsInWorld = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final MultiStepPushRecoveryModule pushRecoveryControlModule;

   public PushRecoveryToolboxController(StatusMessageOutputManager statusOutputManager,
                                        FullHumanoidRobotModel fullRobotModel,
                                        ConvexPolygon2DReadOnly defaultSupportPolygon,
                                        PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                        YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

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

   private class IsInContactProvider implements Function<RobotSide, Boolean>
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
