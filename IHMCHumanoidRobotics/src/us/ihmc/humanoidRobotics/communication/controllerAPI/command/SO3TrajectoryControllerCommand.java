package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class SO3TrajectoryControllerCommand<T extends SO3TrajectoryControllerCommand<T, M>, M extends AbstractSO3TrajectoryMessage<M>> extends QueueableCommand<T, M> implements FrameBasedCommand<M>
{
   private final FrameSO3TrajectoryPointList trajectoryPointList = new FrameSO3TrajectoryPointList();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3,6);
   private ReferenceFrame trajectoryFrame;

   public SO3TrajectoryControllerCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      clearQueuableCommandVariables();
      trajectoryPointList.clear();
      selectionMatrix.reshape(3, 6);
      selectionMatrix.zero();
      for (int i = 0; i < 3; i++)
      {
         selectionMatrix.set(i, i, 1.0);
      }
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      clearQueuableCommandVariables();
      trajectoryPointList.clear(referenceFrame);
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
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
   }
   
   @Override
   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, M message)
   {
      this.trajectoryFrame = trajectoryFrame;
      clear(dataFrame);
      set(message);
   }

   /**
    * Allows setting this orientation {@link #SO3TrajectoryControllerCommand} trajectory command from a pose {@link #SE3TrajectoryControllerCommand}
    * trajectory command.
    */
   public void set(SE3TrajectoryControllerCommand<?, ?> command)
   {
      getTrajectoryPointList().setIncludingFrame(command.getTrajectoryPointList());
      setQueueqableCommandVariables(command);
   }

   /**
    * Same as {@link #set(T)} but does not change the trajectory points.
    * @param other
    */
   public void setPropertiesOnly(T other)
   {
      setQueueqableCommandVariables(other);
      selectionMatrix.set(other.getSelectionMatrix());
   }

   @Override
   public void set(M message)
   {
      message.getTrajectoryPoints(trajectoryPointList);
      setQueueqableCommandVariables(message);
      message.getSelectionMatrix(selectionMatrix);
   }

   public void setTrajectoryPointList(FrameSO3TrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.setIncludingFrame(trajectoryPointList);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public FrameSO3TrajectoryPointList getTrajectoryPointList()
   {
      return trajectoryPointList;
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && trajectoryPointList.getNumberOfTrajectoryPoints() > 0;
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      trajectoryPointList.addTimeOffset(timeOffsetToAdd);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPointList.getNumberOfTrajectoryPoints();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      trajectoryPointList.subtractTimeOffset(timeOffsetToSubtract);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(double time, Quaternion orientation, Vector3D angularVelocity)
   {
      trajectoryPointList.addTrajectoryPoint(time, orientation, angularVelocity);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPointList.addTrajectoryPoint(trajectoryPoint);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public FrameSO3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.getTrajectoryPoint(trajectoryPointIndex);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public FrameSO3TrajectoryPoint getLastTrajectoryPoint()
   {
      return trajectoryPointList.getLastTrajectoryPoint();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      trajectoryPointList.changeFrame(referenceFrame);
   }

   public ReferenceFrame getDataFrame()
   {
      return trajectoryPointList.getReferenceFrame();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void checkReferenceFrameMatch(ReferenceFrame frame)
   {
      trajectoryPointList.checkReferenceFrameMatch(frame);
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return trajectoryFrame;
   }

   public void setTrajectoryFrame(ReferenceFrame trajectoryFrame)
   {
      this.trajectoryFrame = trajectoryFrame;
   }
}
