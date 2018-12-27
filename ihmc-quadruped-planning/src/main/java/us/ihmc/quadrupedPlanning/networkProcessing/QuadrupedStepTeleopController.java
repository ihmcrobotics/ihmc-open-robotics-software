package us.ihmc.quadrupedPlanning.networkProcessing;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.input.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.input.QuadrupedStepTeleopManager;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedStepTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedStepTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicBoolean receivedInput = new AtomicBoolean();

   public QuadrupedStepTeleopController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight, CommandInputManager commandInputManager,
                                        StatusMessageOutputManager statusOutputManager, QuadrupedRobotModelProviderNode robotModelProvider,
                                        YoVariableRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedStepTeleopManager(defaultXGaitSettings, initialBodyHeight, robotModelProvider.getReferenceFrames(),
                                                     Conversions.millisecondsToSeconds(tickTimeMs), graphicsListRegistry, registry);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

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

   public void processTimestamp(long timestampInNanos)
   {
      teleopManager.processTimestamp(timestampInNanos);
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
      statusOutputManager.reportStatusMessage(teleopManager.getStepListMessage());
      statusOutputManager.reportStatusMessage(teleopManager.getBodyOrientationMessage());
      statusOutputManager.reportStatusMessage(teleopManager.getBodyHeightMessage());
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING)
         return true;

      return steppingStateChangeMessage.get().getEndQuadrupedSteppingStateEnum() != QuadrupedSteppingStateChangeMessage.STEP;
   }
}
