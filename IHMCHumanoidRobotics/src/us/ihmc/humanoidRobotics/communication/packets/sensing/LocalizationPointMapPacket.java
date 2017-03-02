package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

public class LocalizationPointMapPacket extends Packet<LocalizationPointMapPacket>
{
   public long timestamp;
   public float[] localizationPointMap;

   public LocalizationPointMapPacket(Random random)
   {
      timestamp = random.nextLong();
      
      int size = Math.abs(random.nextInt(100000));
      
      size = Math.abs(random.nextInt(100000));
      localizationPointMap = new float[size];
      for (int i = 0; i < localizationPointMap.length; i++)
      {
         localizationPointMap[i] = random.nextFloat();
      }
   }
   
   public LocalizationPointMapPacket()
   {
      setDestination(PacketDestination.UI);
   }
   
   public void setLocalizationPointMap(Point3D[] pointCloud)
   {
      localizationPointMap = new float[pointCloud.length*3];
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         localizationPointMap[3 * i] = (float) point.getX();
         localizationPointMap[3 * i + 1] = (float) point.getY();
         localizationPointMap[3 * i + 2] = (float) point.getZ();
      }
   }

   public Point3D32[] getPointMap()
   {
      
      int numberOfPoints = localizationPointMap.length/3;
      
      Point3D32[] points = new Point3D32[numberOfPoints];
      for(int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(localizationPointMap[3 * i]);
         point.setY(localizationPointMap[3 * i + 1]);
         point.setZ(localizationPointMap[3 * i + 2]);
         points[i] = point;
      }
      
      return points;
   }
   
   @Override
   public boolean epsilonEquals(LocalizationPointMapPacket other, double epsilon)
   {
      boolean ret = timestamp == other.timestamp;
      for (int i = 0; i < localizationPointMap.length; i++)
      {
         ret &= localizationPointMap[i] == other.localizationPointMap[i];
      }
      return ret;
   }
   
   @Override
   public String toString()
   {
      return "PointCloudWorldPacket [timestamp=" + timestamp + " points, localizationPointMap=" + localizationPointMap.length/3 + "]";
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

}
