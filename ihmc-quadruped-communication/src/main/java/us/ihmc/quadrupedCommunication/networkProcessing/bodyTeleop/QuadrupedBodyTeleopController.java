package us.ihmc.quadrupedCommunication.networkProcessing.bodyTeleop;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import controller_msgs.msg.dds.QuadrupedTeleopDesiredPose;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyTeleopController extends QuadrupedToolboxController
{
   private final QuadrupedBodyTeleopManager teleopManager;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   public QuadrupedBodyTeleopController(OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver, YoVariableRegistry parentRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      teleopManager = new QuadrupedBodyTeleopManager(robotDataReceiver.getReferenceFrames(), registry);

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

   public void processTeleopDesiredPoseMessage(QuadrupedTeleopDesiredPose message)
   {
      Point3DReadOnly position = message.getPose().getPosition();
      QuaternionReadOnly orientation = message.getPose().getOrientation();
//      teleopManager.setDesiredBodyPose(position.getX(), position.getY(), orientation.getYaw(), orientation.getPitch(), orientation.getRoll(), message.getPoseShiftTime());
      teleopManager.setDesiredBodyPose(0.0, 0.0, orientation.getYaw(), orientation.getPitch(), orientation.getRoll(), message.getPoseShiftTime());
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
      reportMessage(teleopManager.getBodyOrientationMessage());
      reportMessage(teleopManager.getBodyTrajectoryMessage());
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get() == null)
         return false;

      if (controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING)
         return true;

      if (steppingStateChangeMessage.get() == null)
         return false;

      return steppingStateChangeMessage.get().getEndQuadrupedSteppingStateEnum() != QuadrupedSteppingStateEnum.STAND.toByte();
   }
}
