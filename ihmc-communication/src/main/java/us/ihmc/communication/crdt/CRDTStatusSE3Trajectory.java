package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;

public class CRDTStatusSE3Trajectory extends CRDTStatusMutableField<RecyclingArrayList<SE3TrajectoryPoint>>
{
   public CRDTStatusSE3Trajectory(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, () -> new RecyclingArrayList<>(SE3TrajectoryPoint::new));
   }

   public SE3TrajectoryPointReadOnly getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public SE3TrajectoryPointReadOnly getFirstValueReadOnly()
   {
      return getValueInternal().get(0);
   }

   public SE3TrajectoryPointReadOnly getLastValueReadOnly()
   {
      return getValueInternal().get(getSize() - 1);
   }

   public boolean isEmpty()
   {
      return getValueInternal().isEmpty();
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(IDLSequence.Object<SE3TrajectoryPointMessage> trajectoryMessage)
   {
      trajectoryMessage.clear();

      for (SE3TrajectoryPointReadOnly trajectoryPoint : getValueInternal())
      {
         MessageTools.toMessage(trajectoryPoint, trajectoryMessage.add());
      }
   }

   public void fromMessage(IDLSequence.Object<SE3TrajectoryPointMessage> trajectoryMessage)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().clear();

         for (SE3TrajectoryPointMessage trajectoryPointMessage : trajectoryMessage)
         {
            MessageTools.fromMessage(trajectoryPointMessage, getValueInternal().add());
         }
      }
   }

   public void addTrajectoryPoint(RigidBodyTransformReadOnly pose, double time)
   {
      SE3TrajectoryPoint point = accessValue().add();
      point.setTime(time);
      point.getPosition().set(pose.getTranslation());
      point.getOrientation().set(pose.getRotation());
   }

   public void setSingleSegmentTrajectory(RigidBodyTransformReadOnly startPose, RigidBodyTransformReadOnly endPose, double trajectoryDuration)
   {
      accessValue().clear();
      SE3TrajectoryPoint start = accessValue().add();
      start.setTime(0.0);
      start.getPosition().set(startPose.getTranslation());
      start.getOrientation().set(startPose.getRotation());
      SE3TrajectoryPoint end = accessValue().add();
      end.setTime(trajectoryDuration);
      end.getPosition().set(endPose.getTranslation());
      end.getOrientation().set(endPose.getRotation());
   }
}