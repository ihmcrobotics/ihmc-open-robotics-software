package us.ihmc.communication.packets.wholebody;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.robotics.MathTools;

@ClassDocumentation("This message contains a desired joint angle for a single joint.")
public class SingleJointAnglePacket extends IHMCRosApiPacket<SingleJointAnglePacket> implements VisualizablePacket
{   
   public String jointName;
   public double angle;
   public double trajcetoryTime;
   
   @IgnoreField
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
