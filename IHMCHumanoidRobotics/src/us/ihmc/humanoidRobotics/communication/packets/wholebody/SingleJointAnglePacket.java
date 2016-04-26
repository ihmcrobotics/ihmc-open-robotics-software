package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.annotations.ros.RosMessagePacket;
import us.ihmc.communication.annotations.ros.RosIgnoredField;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.robotics.MathTools;

@RosMessagePacket(documentation = "This message contains a desired joint angle for a single joint.",
      rosPackage = "ihmc_msgs",
      topic = "/control/single_joint_angle")
public class SingleJointAnglePacket extends Packet<SingleJointAnglePacket> implements VisualizablePacket
{   
   public String jointName;
   public double angle;
   public double trajcetoryTime;
   
   @RosIgnoredField
   public double resetAngle = Double.NaN;
   
   public SingleJointAnglePacket()
   {
   }
   
   public SingleJointAnglePacket(String jointName, double angle, double trajcetoryTime, double resetAngle)
   {
      this.jointName = jointName;
      this.angle = angle;
      this.trajcetoryTime = trajcetoryTime;
      this.resetAngle = resetAngle;
   }

   @Override
   public boolean epsilonEquals(SingleJointAnglePacket other, double epsilon)
   {
      return other.jointName.equals(this.jointName) && MathTools.epsilonEquals(other.angle, this.angle, epsilon)
            && MathTools.epsilonEquals(other.trajcetoryTime, this.trajcetoryTime, epsilon);
   }
}
