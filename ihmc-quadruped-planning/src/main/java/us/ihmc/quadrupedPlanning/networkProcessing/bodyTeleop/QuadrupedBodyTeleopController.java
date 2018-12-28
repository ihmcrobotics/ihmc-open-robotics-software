package us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import us.ihmc.quadrupedPlanning.networkProcessing.OutputManager;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedBodyTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   public QuadrupedBodyTeleopController(OutputManager statusOutputManager, QuadrupedRobotModelProviderNode robotModelProvider,
                                        YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedBodyTeleopManager(robotModelProvider.getReferenceFrames(), registry);

   }

   public void setPaused(boolean pause)
   {
      teleopManager.setPaused(pause);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processTimestamp(long timestampInNanos)
   {
      teleopManager.processTimestamp(timestampInNanos);
   }

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      teleopManager.setDesiredBodyPose(x, y, yaw, pitch, roll, time);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {
      teleopManager.update();
      reportMessage(teleopManager.getBodyOrientationMessage());
      reportMessage(teleopManager.getBodyTrajectoryMessage());
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING)
         return true;

      return steppingStateChangeMessage.get().getEndQuadrupedSteppingStateEnum() != QuadrupedSteppingStateChangeMessage.STEP;
   }
}
