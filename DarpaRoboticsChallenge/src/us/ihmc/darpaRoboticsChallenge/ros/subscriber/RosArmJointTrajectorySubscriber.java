package us.ihmc.darpaRoboticsChallenge.ros.subscriber;

import java.util.ArrayList;

import trajectory_msgs.JointTrajectory;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.JointTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeTools;
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
   public void onNewMessage(JointTrajectory message)
   {
      RobotSide robotSide;
      if (leftArmNames.equals(message.getJointNames()))
      {
         robotSide = RobotSide.LEFT;
      }
      else if (rightArmNames.equals(message.getJointNames()))
      {
         robotSide = RobotSide.RIGHT;
      }
      else
      {
         System.out.println(getHelp());
         System.err.println(getHelp());
         return;
      }
      
      int numberJoints = message.getJointNames().size();
      int numberPoints = message.getPoints().size();
      
      ArmJointTrajectoryPacket packet = new ArmJointTrajectoryPacket(robotSide, numberPoints, numberJoints);
      for (int i = 0; i < numberPoints; i++)
      {
         double[] positions = message.getPoints().get(i).getPositions();
         double[] velocities = message.getPoints().get(i).getVelocities();
         
         if (positions.length != numberJoints || positions.length != velocities.length)
         {
            String msg = "Number of joints positions or velocities in JointTrajectoryPoint inconsistent with expected number of joints " + numberJoints;
            System.out.println(msg);
            System.err.println(msg);
         }
         
         long nsecs = message.getPoints().get(i).getTimeFromStart().totalNsecs();
         double time = TimeTools.nanoSecondstoSeconds(nsecs);
         
         JointTrajectoryPoint point = new JointTrajectoryPoint(positions, velocities, time);
         packet.trajectoryPoints[i] = point;
      }
      
      packet.setDestination(PacketDestination.CONTROLLER);
      packetCommunicator.send(packet);
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
