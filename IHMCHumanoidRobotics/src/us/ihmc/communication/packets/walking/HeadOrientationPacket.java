package us.ihmc.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationFunctions;

@ClassDocumentation("This message gives the desired head orientation of the robot in world coordinates.")
public class HeadOrientationPacket extends IHMCRosApiPacket<HeadOrientationPacket> implements VisualizablePacket
{
   public Quat4d orientation;
   @FieldDocumentation("trajectoryTime specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;

   public HeadOrientationPacket()
   {
      // Empty constructor for deserialization
   }

   public HeadOrientationPacket(Quat4d orientation, double trajectoryTime)
   {
      this.orientation = orientation;
      this.trajectoryTime = trajectoryTime;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public String toString()
   {
      return this.getClass().getSimpleName() + " " + orientation + " " + trajectoryTime;
   }

   public boolean epsilonEquals(HeadOrientationPacket other, double epsilon)
   {
      boolean ret = RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      ret &= MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime, epsilon);
      return ret;
   }

   public HeadOrientationPacket(Random random)
   {
      this.orientation = RandomTools.generateRandomQuaternion(random);
      this.trajectoryTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1);
   }
}
