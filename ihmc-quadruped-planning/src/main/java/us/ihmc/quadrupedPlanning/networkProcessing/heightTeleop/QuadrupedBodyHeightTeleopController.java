package us.ihmc.quadrupedPlanning.networkProcessing.heightTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyHeightTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedBodyHeightTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();

   private final AtomicBoolean receivedInput = new AtomicBoolean();

   public QuadrupedBodyHeightTeleopController(double initialBodyHeight, CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              QuadrupedRobotModelProviderNode robotModelProvider, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedBodyHeightTeleopManager(initialBodyHeight, robotModelProvider.getReferenceFrames());

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));
   }

   public void setPaused(boolean pause)
   {
      teleopManager.setPaused(pause);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      teleopManager.setDesiredBodyHeight(desiredBodyHeight);
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
      statusOutputManager.reportStatusMessage(teleopManager.getBodyHeightMessage());
   }

   @Override
   public boolean isDone()
   {
      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }
}
