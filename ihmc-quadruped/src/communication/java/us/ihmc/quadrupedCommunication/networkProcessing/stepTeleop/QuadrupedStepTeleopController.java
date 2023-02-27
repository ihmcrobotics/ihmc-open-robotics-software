package us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.AbortWalkingMessage;
import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import quadruped_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import quadruped_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import quadruped_msgs.msg.dds.QuadrupedTeleopDesiredVelocity;
import quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedStepTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedStepTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   public QuadrupedStepTeleopController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                        OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver, YoRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedStepTeleopManager(defaultXGaitSettings, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(),
                                                     Conversions.millisecondsToSeconds(tickTimeMs), graphicsListRegistry, registry);
   }

   public void setPaused(boolean pause)
   {
      teleopManager.setPaused(pause);

      if(pause)
      {
         reportMessage(new AbortWalkingMessage());
      }
   }

   public void processBodyPathPlanMessage(QuadrupedBodyPathPlanMessage message)
   {
      teleopManager.processBodyPathPlanMessage(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      teleopManager.processFootstepStatusMessage(message);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      teleopManager.processPlanarRegionsListMessage(message);
   }

   public void processGroundPlaneMessage(GroundPlaneMessage message)
   {
      teleopManager.processGroundPlaneMessage(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      teleopManager.processXGaitSettingsPacket(packet);
   }

   public void processTimestamp(long timestampInNanos)
   {
      teleopManager.processTimestamp(timestampInNanos);
   }

   public void processTeleopDesiredVelocity(QuadrupedTeleopDesiredVelocity message)
   {
      teleopManager.setDesiredVelocity(message.getDesiredXVelocity(), message.getDesiredYVelocity(), message.getDesiredYawVelocity());
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      teleopManager.setShiftPlanBasedOnStepAdjustment(shift);
   }

   @Override
   public boolean initializeInternal()
   {
      teleopManager.initialize();

      return true;
   }

   @Override
   public void updateInternal()
   {
      teleopManager.update();
      reportMessage(teleopManager.getStepListMessage());
      reportMessage(teleopManager.getBodyOrientationMessage());
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get() == null)
         return false;

      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }
}
