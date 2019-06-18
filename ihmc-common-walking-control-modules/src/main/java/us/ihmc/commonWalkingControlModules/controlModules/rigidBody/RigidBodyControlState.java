package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class RigidBodyControlState implements State
{
   protected final YoVariableRegistry registry;
   protected final String warningPrefix;

   protected final YoBoolean trajectoryDone;

   private final YoLong lastCommandId;
   private final YoDouble trajectoryStartTime;
   private final YoDouble yoTime;

   protected final ArrayList<YoGraphic> graphics = new ArrayList<>();
   private final RigidBodyControlMode controlMode;

   public RigidBodyControlState(RigidBodyControlMode controlMode, String bodyName, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      this.controlMode = controlMode;
      this.yoTime = yoTime;

      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      registry = new YoVariableRegistry(createRegistryName(bodyName, controlMode));

      String prefix;
      if (controlMode != null)
         prefix = bodyName + StringUtils.capitalize(controlMode.toString().toLowerCase());
      else
         prefix = bodyName;

      lastCommandId = new YoLong(prefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      trajectoryDone = new YoBoolean(prefix + "TrajectoryDone", registry);
      trajectoryStartTime = new YoDouble(prefix + "TrajectoryStartTime", registry);

      parentRegistry.addChild(registry);
   }

   protected boolean handleCommandInternal(Command<?, ?> command)
   {
      if (command instanceof QueueableCommand<?, ?>)
      {
         QueueableCommand<?, ?> queueableCommand = (QueueableCommand<?, ?>) command;

         if (queueableCommand.getCommandId() == Packet.INVALID_MESSAGE_ID)
         {
            LogTools.warn(warningPrefix + "Recieved packet with invalid ID.");
            return false;
         }

         boolean wantToQueue = queueableCommand.getExecutionMode() == ExecutionMode.QUEUE;
         boolean previousIdMatch = queueableCommand.getPreviousCommandId() == lastCommandId.getLongValue();

         if (!isEmpty() && wantToQueue && !previousIdMatch)
         {
            LogTools.warn(warningPrefix + "Unexpected command ID. Msg previous id: " + queueableCommand.getPreviousCommandId() + " but was "
                  + lastCommandId.getLongValue());
            return false;
         }

         if (!wantToQueue || isEmpty())
            setTrajectoryStartTimeToCurrentTime();
         else
            queueableCommand.addTimeOffset(getLastTrajectoryPointTime());

         lastCommandId.set(queueableCommand.getCommandId());
      }
      else
      {
         setTrajectoryStartTimeToCurrentTime();
      }

      trajectoryDone.set(false);
      return true;
   }

   protected void setTrajectoryStartTimeToCurrentTime()
   {
      trajectoryStartTime.set(yoTime.getDoubleValue());
   }

   public double getTimeInTrajectory()
   {
      return yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue();
   }

   protected void resetLastCommandId()
   {
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);
   }

   public boolean abortState()
   {
      return false;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public abstract boolean isEmpty();

   public abstract double getLastTrajectoryPointTime();

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }

   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      return null;
   }

   protected void updateGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
         graphics.get(graphicsIdx).update();
   }

   protected void hideGraphics()
   {
      // TODO: make a hide method in the YoGraphic or find some other way to avoid this mess.
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
      {
         YoGraphic yoGraphic = graphics.get(graphicsIdx);
         if (yoGraphic instanceof YoGraphicReferenceFrame)
            ((YoGraphicReferenceFrame) yoGraphic).hide();
         else if (yoGraphic instanceof YoGraphicPosition)
            ((YoGraphicPosition) yoGraphic).setPositionToNaN();
         else if (yoGraphic instanceof YoGraphicVector)
            ((YoGraphicVector) yoGraphic).hide();
         else if (yoGraphic instanceof YoGraphicCoordinateSystem)
            ((YoGraphicCoordinateSystem) yoGraphic).hide();
         else
            throw new RuntimeException("Implement hiding this.");
      }
   }

   public static String createRegistryName(String bodyName, RigidBodyControlMode stateEnum)
   {
      String prefix;
      if (stateEnum != null)
         prefix = bodyName + StringUtils.capitalize(stateEnum.toString().toLowerCase());
      else
         prefix = bodyName;
      return prefix + "ControlModule";
   }

   public RigidBodyControlMode getControlMode()
   {
      return controlMode;
   }

   public Object pollStatusToReport()
   {
      return null;
   }
}
