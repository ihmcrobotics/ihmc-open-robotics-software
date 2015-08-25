package us.ihmc.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("Packet for executing an arm joint trajectory. It works similar to the\n"
                                  + "trajectory_msgs/JointTrajectory message.")
public class ArmJointTrajectoryPacket extends IHMCRosApiPacket<ArmJointTrajectoryPacket> implements VisualizablePacket
{
   @FieldDocumentation("Specifies the side of the robot that will execute the trajectory")
   public RobotSide robotSide;
   @FieldDocumentation("List of points in the trajectory")
   public JointTrajectoryPoint[] trajectoryPoints;
   
   public ArmJointTrajectoryPacket()
   {
   }
   
   public ArmJointTrajectoryPacket(ArmJointTrajectoryPacket armJointTrajectoryPacket)
   {
      this.robotSide = armJointTrajectoryPacket.robotSide;
      
      if (armJointTrajectoryPacket.trajectoryPoints != null)
      {
         this.trajectoryPoints = new JointTrajectoryPoint[armJointTrajectoryPacket.trajectoryPoints.length];
         for (int i=0; i<armJointTrajectoryPacket.trajectoryPoints.length; i++)
         {
            this.trajectoryPoints[i] = new JointTrajectoryPoint(armJointTrajectoryPacket.trajectoryPoints[i]);
         }
      }
         
      
   }
   
   public ArmJointTrajectoryPacket(RobotSide robotSide, JointTrajectoryPoint[] trajectoryPoints)
   {
      this.robotSide = robotSide;
      this.trajectoryPoints = trajectoryPoints;
   }
   
   public ArmJointTrajectoryPacket(RobotSide robotSide, int waypoints, int armJoints)
   {
      this.robotSide = robotSide;
      this.trajectoryPoints = new JointTrajectoryPoint[waypoints];
      for (int i = 0; i < waypoints; i++)
      {
         trajectoryPoints[i] = new JointTrajectoryPoint(armJoints);
      }
   }
   
   @Override
   public boolean epsilonEquals(ArmJointTrajectoryPacket other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide) || this.trajectoryPoints.length != other.trajectoryPoints.length)
      {
         return false;
      }
      
      for (int i = 0; i < this.trajectoryPoints.length; i++)
      {
         if (!this.trajectoryPoints[i].epsilonEquals(other.trajectoryPoints[i], epsilon))
         {
            return false;
         }
      }
      
      return true;
   }
   
   public ArmJointTrajectoryPacket(Random random)
   {
      this(random, random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, random.nextInt(16) + 1);
   }
   public ArmJointTrajectoryPacket(Random random, RobotSide robotSide, int numberOfPoints)
   {
      this.robotSide = robotSide;
      this.trajectoryPoints = new JointTrajectoryPoint[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         trajectoryPoints[i] = new JointTrajectoryPoint(random);
      }
   }

   @Override
   public String toString()
   {
      return "ArmJointTrajectoryPacket [robotSide=" + robotSide + ", trajectoryPoints=" + Arrays.toString(trajectoryPoints) + "]";
   }
}
