package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.tools.io.printing.PrintTools;

public abstract class RigidBodyControlState extends FinishableState<RigidBodyControlMode>
{
   protected final YoVariableRegistry registry;
   protected final String warningPrefix;

   protected final BooleanYoVariable trajectoryStopped;
   protected final BooleanYoVariable trajectoryDone;

   private final LongYoVariable lastCommandId;
   private final DoubleYoVariable trajectoryStartTime;
   private final DoubleYoVariable yoTime;

   public RigidBodyControlState(RigidBodyControlMode stateEnum, String bodyName, DoubleYoVariable yoTime)
   {
      super(stateEnum);
      this.yoTime = yoTime;

      String prefix = bodyName + StringUtils.capitalize(stateEnum.toString().toLowerCase());
      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      registry = new YoVariableRegistry(prefix + "ControlModule");
      lastCommandId = new LongYoVariable(prefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      trajectoryStopped = new BooleanYoVariable(prefix + "TrajectoryStopped", registry);
      trajectoryDone = new BooleanYoVariable(prefix + "TrajectoryDone", registry);
      trajectoryStartTime = new DoubleYoVariable(prefix + "TrajectoryStartTime", registry);
   }

   protected boolean handleCommandInternal(QueueableCommand<?, ?> command)
   {
      if (command.getCommandId() == Packet.INVALID_MESSAGE_ID)
      {
         PrintTools.warn(warningPrefix + "Recieved packet with invalid ID.");
         return false;
      }

      boolean wantToQueue = command.getExecutionMode() == ExecutionMode.QUEUE;
      boolean previousIdMatch = command.getPreviousCommandId() == lastCommandId.getLongValue();

      if (!isEmpty() && wantToQueue && !previousIdMatch)
      {
         PrintTools.warn(warningPrefix + "Unexpected command ID.");
         return false;
      }

      if (!wantToQueue || isEmpty())
         trajectoryStartTime.set(yoTime.getDoubleValue());
      else
         command.addTimeOffset(getLastTrajectoryPointTime());

      lastCommandId.set(command.getCommandId());
      trajectoryStopped.set(false);
      trajectoryDone.set(false);
      return true;
   }

   protected double getTimeInTrajectory()
   {
      return yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue();
   }

   protected void resetLastCommandId()
   {
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      trajectoryStopped.set(command.isStopAllTrajectory());
   }

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract boolean isEmpty();

   public abstract double getLastTrajectoryPointTime();

   @Override
   public boolean isDone()
   {
      return true;
   }
}
