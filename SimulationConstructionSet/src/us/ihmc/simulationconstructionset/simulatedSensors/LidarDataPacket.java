package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;


@HighBandwidthPacket
public class LidarDataPacket extends Packet<LidarDataPacket>
{
   public final ArrayList<Point3d> points = new ArrayList<Point3d>();
   
   public LidarDataPacket()
   {
      
   }

   public boolean epsilonEquals(LidarDataPacket other, double epsilon)
   {
      return points.equals(other);
   }

   public void add(Point3d point3d)
   {
      points.add(point3d);
   }
}
