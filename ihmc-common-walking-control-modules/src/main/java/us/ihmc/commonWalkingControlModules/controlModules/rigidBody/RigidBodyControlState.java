package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;

/**
 * Holds the functionality common to all the rigid body control states.
 */
public abstract class RigidBodyControlState implements State, SCS2YoGraphicHolder
{
   protected final YoRegistry registry;
   protected final String warningPrefix;

   protected final YoBoolean trajectoryDone;

   private final YoLong lastCommandId;
   private final YoDouble trajectoryStartTime;
   private final YoDouble yoTime;

   private final RigidBodyControlMode controlMode;

   public RigidBodyControlState(RigidBodyControlMode controlMode, String bodyName, YoDouble yoTime, YoRegistry parentRegistry)
   {
      this.controlMode = controlMode;
      this.yoTime = yoTime;

      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      registry = new YoRegistry(createRegistryName(bodyName, controlMode));

      String prefix;
      if (controlMode != null)
         prefix = bodyName + StringUtils.capitalize(controlMode.toString().toLowerCase());
      else
         prefix = bodyName;

      lastCommandId = new YoLong(prefix + "LastCommandId", registry);
      lastCommandId.set(QueueableCommand.INVALID_MESSAGE_ID);

      trajectoryDone = new YoBoolean(prefix + "TrajectoryDone", registry);
      trajectoryStartTime = new YoDouble(prefix + "TrajectoryStartTime", registry);

      parentRegistry.addChild(registry);
   }

   protected boolean handleCommandInternal(Command<?, ?> command)
   {
      if (command instanceof QueueableCommand<?, ?>)
      {
         QueueableCommand<?, ?> queueableCommand = (QueueableCommand<?, ?>) command;

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
      lastCommandId.set(QueueableCommand.INVALID_MESSAGE_ID);
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

   public JointDesiredOutputListReadOnly getJointDesiredData()
   {
      return null;
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
