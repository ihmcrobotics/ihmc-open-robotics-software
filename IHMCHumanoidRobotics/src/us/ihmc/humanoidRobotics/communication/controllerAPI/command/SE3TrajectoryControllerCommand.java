package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class SE3TrajectoryControllerCommand<T extends SE3TrajectoryControllerCommand<T, M>, M extends AbstractSE3TrajectoryMessage<M>> extends QueueableCommand<T, M>
{
   private final FrameSE3TrajectoryPointList trajectoryPointList = new FrameSE3TrajectoryPointList();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(6);

   public SE3TrajectoryControllerCommand()
   {
   }

   @Override
   public void clear()
   {
      clearQueuableCommandVariables();
      selectionMatrix.reshape(6, 6);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      trajectoryPointList.clear(referenceFrame);
      clearQueuableCommandVariables();
      selectionMatrix.reshape(6, 6);
      CommonOps.setIdentity(selectionMatrix);
   }

   @Override
   public void set(T other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
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

   public void setTrajectoryPointList(FrameSE3TrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.setIncludingFrame(trajectoryPointList);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public FrameSE3TrajectoryPointList getTrajectoryPointList()
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
   public void addTrajectoryPoint(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      trajectoryPointList.addTrajectoryPoint(time, position, orientation, linearVelocity, angularVelocity);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPointList.addTrajectoryPoint(trajectoryPoint);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public FrameSE3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.getTrajectoryPoint(trajectoryPointIndex);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public FrameSE3TrajectoryPoint getLastTrajectoryPoint()
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

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use {@link #getTrajectoryPointList()}.
    */
   public ReferenceFrame getReferenceFrame()
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
}
