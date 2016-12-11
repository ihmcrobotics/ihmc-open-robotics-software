package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class SO3TrajectoryControllerCommand<T extends SO3TrajectoryControllerCommand<T, M>, M extends AbstractSO3TrajectoryMessage<M>>
      extends FrameSO3TrajectoryPointList implements Command<T, M>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3,6);

   public SO3TrajectoryControllerCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      super.clear();
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = ExecutionMode.OVERRIDE;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
      selectionMatrix.reshape(3, 6);
      selectionMatrix.zero();
      for (int i = 0; i < 3; i++)
      {
         selectionMatrix.set(i, i, 1.0);
      }
   }

   @Override
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = ExecutionMode.OVERRIDE;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
      selectionMatrix.reshape(3, 6);
      selectionMatrix.zero();
      for (int i = 0; i < 3; i++)
      {
         selectionMatrix.set(i, i, 1.0);
      }
   }

   @Override
   public void set(T other)
   {
      setIncludingFrame(other);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(T)} but does not change the trajectory points.
    * @param other
    */
   public void setPropertiesOnly(T other)
   {
      commandId = other.getCommandId();
      executionMode = other.getExecutionMode();
      previousCommandId = other.getPreviousCommandId();
      selectionMatrix.set(other.getSelectionMatrix());
   }

   @Override
   public void set(M message)
   {
      message.getTrajectoryPoints(this);
      commandId = message.getUniqueId();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
      message.getSelectionMatrix(selectionMatrix);
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public void setPreviousCommandId(long previousCommandId)
   {
      this.previousCommandId = previousCommandId;
   }

   public long getCommandId()
   {
      return commandId;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousCommandId()
   {
      return previousCommandId;
   }
   
   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public boolean isCommandValid()
   {
      return executionMode != null && getNumberOfTrajectoryPoints() > 0;
   }
}
