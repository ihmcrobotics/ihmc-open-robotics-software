package us.ihmc.avatar.ros.subscriber;

import java.util.ArrayList;

import trajectory_msgs.JointTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RosArmJointTrajectorySubscriber extends AbstractRosTopicSubscriber<trajectory_msgs.JointTrajectory>
{
   private final PacketCommunicator packetCommunicator;
   private final ArrayList<String> leftArmNames = new ArrayList<String>();
   private final ArrayList<String> rightArmNames = new ArrayList<String>();

   public RosArmJointTrajectorySubscriber(PacketCommunicator controllerCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(trajectory_msgs.JointTrajectory._TYPE);
      this.packetCommunicator = controllerCommunicationBridge;
      
      ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      
      for (int i = 0; i < armJointNames.length; i++)
      {
         ArmJointName jointName = armJointNames[i];
         leftArmNames.add(fullRobotModel.getArmJoint(RobotSide.LEFT, jointName).getName());
         rightArmNames.add(fullRobotModel.getArmJoint(RobotSide.RIGHT, jointName).getName());
      }
   }

   @Override
   public void onNewMessage(JointTrajectory rosMessage)
   {
      RobotSide robotSide;
      if (leftArmNames.equals(rosMessage.getJointNames()))
      {
         robotSide = RobotSide.LEFT;
      }
      else if (rightArmNames.equals(rosMessage.getJointNames()))
      {
         robotSide = RobotSide.RIGHT;
      }
      else
      {
         System.out.println(getHelp());
         System.err.println(getHelp());
         return;
      }
      
      int numberOfJoints = rosMessage.getJointNames().size();
      int numberOfWaypoints = rosMessage.getPoints().size();
      
      ArmTrajectoryMessage ihmcMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfWaypoints);
      for (int waypointIndex = 0; waypointIndex < numberOfWaypoints; waypointIndex++)
      {
         double[] positions = rosMessage.getPoints().get(waypointIndex).getPositions();
         double[] velocities = rosMessage.getPoints().get(waypointIndex).getVelocities();
         
         if (positions.length != numberOfJoints || positions.length != velocities.length)
         {
            String msg = "Number of joints positions or velocities in JointTrajectoryPoint inconsistent with expected number of joints " + numberOfJoints;
            System.out.println(msg);
            System.err.println(msg);
         }
         
         long nsecs = rosMessage.getPoints().get(waypointIndex).getTimeFromStart().totalNsecs();
         double time = Conversions.nanoSecondstoSeconds(nsecs);
         
         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            ihmcMessage.setTrajectoryPoint(jointIndex, waypointIndex, time, positions[jointIndex], velocities[jointIndex]);
         }
      }
      
      ihmcMessage.setDestination(PacketDestination.CONTROLLER);
      packetCommunicator.send(ihmcMessage);
   }
   
   private String getHelp()
   {
      String helpText = "Expected arm joint names for the left side are:\n";
      for (int i = 0; i < leftArmNames.size(); i++)
      {
         helpText += leftArmNames.get(i) + " ";
      }
      helpText += "\nFor a right side trajectory please provide:\n";
      for (int i = 0; i < rightArmNames.size(); i++)
      {
         helpText += rightArmNames.get(i) + " ";
      }
      helpText += "\nIt is required to provide the full list of joint angles in the correct order.";
      
      return helpText;
   }
}
