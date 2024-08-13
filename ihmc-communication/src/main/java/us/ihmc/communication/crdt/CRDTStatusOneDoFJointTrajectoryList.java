package us.ihmc.communication.crdt;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointReadOnly;

public class CRDTStatusOneDoFJointTrajectoryList extends CRDTStatusMutableField<RecyclingArrayList<RecyclingArrayList<OneDoFTrajectoryPoint>>>
{
   public CRDTStatusOneDoFJointTrajectoryList(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, () -> new RecyclingArrayList<>(() -> new RecyclingArrayList<>(OneDoFTrajectoryPoint::new)));
   }

   public OneDoFTrajectoryPointReadOnly getValueReadOnly(int jointIndex, int trajectoryPointIndex)
   {
      return getValueInternal().get(jointIndex).get(trajectoryPointIndex);
   }

   public OneDoFTrajectoryPointReadOnly getFirstValueReadOnly(int jointIndex)
   {
      return getValueInternal().get(jointIndex).get(0);
   }

   public OneDoFTrajectoryPointReadOnly getLastValueReadOnly(int jointIndex)
   {
      return getValueInternal().get(jointIndex).get(getNumberOfJoints() - 1);
   }

   public boolean isEmpty()
   {
      return getValueInternal().isEmpty();
   }

   public int getNumberOfJoints()
   {
      return getValueInternal().size();
   }

   public int getNumberOfPoints(int jointIndex)
   {
      return getValueInternal().get(jointIndex).size();
   }

   public void toMessage(IDLSequence.Object<OneDoFJointTrajectoryMessage> trajectoryMessage)
   {
      trajectoryMessage.clear();

      for (RecyclingArrayList<OneDoFTrajectoryPoint> oneDoFTrajectory : getValueInternal())
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = trajectoryMessage.add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();

         for (OneDoFTrajectoryPoint oneDoFTrajectoryPoint : oneDoFTrajectory)
         {
            MessageTools.toMessage(oneDoFTrajectoryPoint, oneDoFJointTrajectoryMessage.getTrajectoryPoints().add());
         }
      }
   }

   public void fromMessage(IDLSequence.Object<OneDoFJointTrajectoryMessage> trajectoryMessage)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().clear();

         for (OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage : trajectoryMessage)
         {
            RecyclingArrayList<OneDoFTrajectoryPoint> trajectoryPointList = getValueInternal().add();
            trajectoryPointList.clear();

            for (TrajectoryPoint1DMessage trajectoryPointMessage : oneDoFJointTrajectoryMessage.getTrajectoryPoints())
            {
               MessageTools.fromMessage(trajectoryPointMessage, trajectoryPointList.add());
            }
         }
      }
   }

   public void clear(int numberOfJoints)
   {
      for (RecyclingArrayList<OneDoFTrajectoryPoint> oneDoFTrajectoryPoints : accessValue())
      {
         oneDoFTrajectoryPoints.clear();
      }

      accessValue().clear();
      for (int i = 0; i < numberOfJoints; i++)
         accessValue().add();
   }

   public void addTrajectoryPoint(int jointIndex, double position, double time)
   {
      OneDoFTrajectoryPoint point = accessValue().get(jointIndex).add();
      point.setTime(time);
      point.setPosition(position);
   }

   public void setSingleSegmentTrajectory(int jointIndex, double startPosition, double endPosition, double trajectoryDuration)
   {
      accessValue().clear();
      OneDoFTrajectoryPoint start = accessValue().get(jointIndex).add();
      start.setTime(0.0);
      start.setPosition(startPosition);
      OneDoFTrajectoryPoint end = accessValue().get(jointIndex).add();
      end.setTime(trajectoryDuration);
      end.setPosition(endPosition);
   }
}