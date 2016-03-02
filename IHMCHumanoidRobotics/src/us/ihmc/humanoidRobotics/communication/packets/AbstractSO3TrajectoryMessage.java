package us.ihmc.humanoidRobotics.communication.packets;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;

public abstract class AbstractSO3TrajectoryMessage<T extends AbstractSO3TrajectoryMessage<T>> extends IHMCRosApiMessage<T>
      implements TransformableDataObject<T>, TrajectoryPointListInterface<SO3TrajectoryPointMessage, T>, Transformable
{
   @FieldDocumentation("List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SO3TrajectoryPointMessage[] taskspaceTrajectoryPoints;

   public AbstractSO3TrajectoryMessage()
   {
   }

   public AbstractSO3TrajectoryMessage(T so3TrajectoryMessage)
   {
      taskspaceTrajectoryPoints = new SO3TrajectoryPointMessage[so3TrajectoryMessage.getNumberOfTrajectoryPoints()];
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SO3TrajectoryPointMessage(so3TrajectoryMessage.taskspaceTrajectoryPoints[i]);
   }

   public AbstractSO3TrajectoryMessage(double trajectoryTime, Quat4d desiredOrientation)
   {
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceTrajectoryPoints = new SO3TrajectoryPointMessage[] {new SO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, zeroAngularVelocity)};
   }

   public AbstractSO3TrajectoryMessage(int numberOfTrajectoryPoints)
   {
      taskspaceTrajectoryPoints = new SO3TrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   @Override
   public final void clear()
   {
      throw new RuntimeException("Cannot clear a trajectory message.");
   }

   @Override
   public final void addTrajectoryPoint(SO3TrajectoryPointMessage trajectoryPoint)
   {
      throw new RuntimeException("Cannot add a trajectory point to a trajectory message.");
   }

   @Override
   public void set(T other)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         throw new RuntimeException("Must the same number of waypoints.");
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SO3TrajectoryPointMessage(other.taskspaceTrajectoryPoints[i]);
   }

   /**
    * Create a trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param orientation define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Quat4d orientation, Vector3d angularVelocity)
   {
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new SO3TrajectoryPointMessage(time, orientation, angularVelocity);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i].applyTransform(transform);
   }

   @Override
   public final int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.length;
   }

   public final SO3TrajectoryPointMessage[] getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   @Override
   public final SO3TrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints[trajectoryPointIndex];
   }

   @Override
   public final SO3TrajectoryPointMessage getLastTrajectoryPoint()
   {
      return taskspaceTrajectoryPoints[taskspaceTrajectoryPoints.length - 1];
   }

   @Override
   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().time;
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException(
               "Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: " + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!taskspaceTrajectoryPoints[i].epsilonEquals(other.taskspaceTrajectoryPoints[i], epsilon))
            return false;
      }

      return true;
   }
}
