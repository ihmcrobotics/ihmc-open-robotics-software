package us.ihmc.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.robotics.MathTools;
import us.ihmc.tools.random.RandomTools;

@ClassDocumentation("This message sets the robot's center of mass height.")
public class ComHeightPacket extends IHMCRosApiPacket<ComHeightPacket> implements VisualizablePacket
{
   public static final double MIN_COM_HEIGHT = -0.33; //-0.40
   public static final double MAX_COM_HEIGHT = 0.14;

   @FieldDocumentation("heightOffset specifies CoM height relative to the default starting height, which is\n"
                                     + "about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about\n"
                                     + "68.9 cm above ground level")
   public double heightOffset;
   @FieldDocumentation("trajectoryTime specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;

   public ComHeightPacket()
   {
   }

   public ComHeightPacket(double heightOffset)
   {
      this(heightOffset, 1.0);
   }

   public ComHeightPacket(double heightOffset, double trajectoryTime)
   {
      this.heightOffset = MathTools.clipToMinMax(heightOffset, MIN_COM_HEIGHT, MAX_COM_HEIGHT);
      this.trajectoryTime = trajectoryTime;
   }

   public double getHeightOffset()
   {
      return heightOffset;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public String toString()
   {
      return "heightOffset = " + heightOffset + "trajectory time =  " + trajectoryTime;
   }

   @Override
   public boolean epsilonEquals(ComHeightPacket other, double epsilon)
   {
      boolean ret = MathTools.epsilonEquals(other.getHeightOffset(), this.getHeightOffset(), epsilon);
      ret &= MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime, epsilon);
      return ret;
   }

   public ComHeightPacket(Random random)
   {
      this(RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1), RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1));
   }

}
