package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosIgnoredField;

public class SingleJointAnglePacket extends Packet<SingleJointAnglePacket>
{   
   public String jointName;
   public double angle;
   public double trajectoryTime;
   
   @RosIgnoredField
   public double resetAngle = Double.NaN;
   
   public SingleJointAnglePacket()
   {
   }
   
   public SingleJointAnglePacket(String jointName, double angle, double trajectoryTime, double resetAngle)
   {
      this.jointName = jointName;
      this.angle = angle;
      this.trajectoryTime = trajectoryTime;
      this.resetAngle = resetAngle;
   }

   @Override
   public boolean epsilonEquals(SingleJointAnglePacket other, double epsilon)
   {
      return other.jointName.equals(this.jointName) && MathTools.epsilonEquals(other.angle, this.angle, epsilon)
            && MathTools.epsilonEquals(other.trajectoryTime, this.trajectoryTime, epsilon);
   }
}
