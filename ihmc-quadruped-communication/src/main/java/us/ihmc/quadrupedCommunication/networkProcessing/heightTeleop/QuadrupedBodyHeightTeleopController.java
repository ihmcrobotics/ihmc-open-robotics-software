package us.ihmc.quadrupedCommunication.networkProcessing.heightTeleop;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyHeightTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedBodyHeightTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();

   public QuadrupedBodyHeightTeleopController(double initialBodyHeight, OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                              YoVariableRegistry parentRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedBodyHeightTeleopManager(initialBodyHeight, robotDataReceiver.getReferenceFrames());
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
   public boolean initializeInternal()
   {
      teleopManager.initialize();

      return true;
   }

   @Override
   public void updateInternal()
   {
      teleopManager.update();
      reportMessage(teleopManager.getBodyHeightMessage());
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get() == null)
         return false;

      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }
}
