package us.ihmc.quadrupedPlanning.networkProcessing.stepTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.networkProcessing.OutputManager;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedStepTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedStepTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   public QuadrupedStepTeleopController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, OutputManager statusOutputManager,
                                        QuadrupedRobotModelProviderNode robotModelProvider, YoVariableRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedStepTeleopManager(defaultXGaitSettings, robotModelProvider.getReferenceFrames(),
                                                     Conversions.millisecondsToSeconds(tickTimeMs), graphicsListRegistry, registry);
   }

   public void setPaused(boolean pause)
   {
      teleopManager.setPaused(pause);
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

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
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

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      teleopManager.setDesiredVelocity(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      teleopManager.setEndPhaseShift(endPhaseShift);
   }

   @Override
   public boolean initialize()
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
      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }
}
