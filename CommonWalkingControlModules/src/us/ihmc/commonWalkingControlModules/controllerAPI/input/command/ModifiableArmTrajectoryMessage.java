package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPoint1DInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableArmTrajectoryMessage
{
   private final RecyclingArrayList<SimpleTrajectoryPoint1DList> jointTrajectoryInputs = new RecyclingArrayList<>(10, SimpleTrajectoryPoint1DList.class);
   private RobotSide robotSide;

   public ModifiableArmTrajectoryMessage()
   {
      clear();
   }

   public void clear()
   {
      clear(null);
   }

   public void clear(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      jointTrajectoryInputs.clear();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void addArmTrajectoryPoint(double trajectoryPointTime, double[] desiredJointPositions, double[] desiredJointVelocities)
   {
      MathTools.checkIfEqual(desiredJointPositions.length, desiredJointVelocities.length);
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         SimpleTrajectoryPoint1DList jointTrajectoryInput = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
         jointTrajectoryInput.addTrajectoryPoint(trajectoryPointTime, desiredJointPositions[jointIndex], desiredJointVelocities[jointIndex]);
      }
   }

   public void set(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      clear(robotSide);
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         SimpleTrajectoryPoint1DList jointTrajectoryInput = jointTrajectoryInputs.add();
         jointTrajectoryInput.clear();
         jointTrajectoryInput.addTrajectoryPoint(trajectoryTime, desiredJointPositions[jointIndex], 0.0);
      }
   }

   public void set(ArmTrajectoryMessage armTrajectoryMessage)
   {
      set(armTrajectoryMessage.getRobotSide(), armTrajectoryMessage.getTrajectoryPointLists());
   }

   public <T extends TrajectoryPointListInterface<? extends TrajectoryPoint1DInterface<?>, T>> void set(RobotSide robotSide, T[] trajectoryPointListArray)
   {
      clear(robotSide);
      for (int i = 0; i < trajectoryPointListArray.length; i++)
      {
         jointTrajectoryInputs.add().set(trajectoryPointListArray[i]);
         set(i, trajectoryPointListArray[i]);
      }
   }

   public <T extends TrajectoryPointListInterface<? extends TrajectoryPoint1DInterface<?>, T>> void set(int jointIndex, T otherTrajectoryPointList)
   {
      SimpleTrajectoryPoint1DList thisJointTrajectoryPointList = jointTrajectoryInputs.get(jointIndex);
      thisJointTrajectoryPointList.set(otherTrajectoryPointList);
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryInputs.size();
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getNumberOfTrajectoryPoints();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public RecyclingArrayList<SimpleTrajectoryPoint1DList> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public SimpleTrajectoryPoint1D getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public SimpleTrajectoryPoint1DList getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
   }
}
