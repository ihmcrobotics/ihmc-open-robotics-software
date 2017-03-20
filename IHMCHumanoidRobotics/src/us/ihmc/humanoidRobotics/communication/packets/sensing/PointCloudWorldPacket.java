package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
@HighBandwidthPacket
public class PointCloudWorldPacket extends Packet<PointCloudWorldPacket>
{
   public long timestamp;
   
   public float[] groundQuadTreeSupport;
   
   // Code is duplicated, probably gets replaced with locality hash
   public float[] decayingWorldScan;

   public float defaultGroundHeight;
   
   public PointCloudWorldPacket(Random random)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      timestamp = random.nextLong();
      
      int size = Math.abs(random.nextInt(100000));
      groundQuadTreeSupport = new float[size];
      for (int i = 0; i < groundQuadTreeSupport.length; i++)
      {
         groundQuadTreeSupport[i] = random.nextFloat();
      }
      
      size = Math.abs(random.nextInt(100000));
      decayingWorldScan = new float[size];
      for (int i = 0; i < decayingWorldScan.length; i++)
      {
         decayingWorldScan[i] = random.nextFloat();
      }
   }
   
   public PointCloudWorldPacket()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDestination(PacketDestination.BROADCAST);
   }
   
   public void setGroundQuadTreeSupport(Point3D[] pointCloud)
   {
      groundQuadTreeSupport = new float[pointCloud.length*3];
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         groundQuadTreeSupport[3 * i] = (float) point.getX();
         groundQuadTreeSupport[3 * i + 1] = (float) point.getY();
         groundQuadTreeSupport[3 * i + 2] = (float) point.getZ();
      }
   }
   
   public void setDecayingWorldScan(Point3D[] pointCloud)
   {
      decayingWorldScan = new float[pointCloud.length*3];
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         decayingWorldScan[3 * i] = (float) point.getX();
         decayingWorldScan[3 * i + 1] = (float) point.getY();
         decayingWorldScan[3 * i + 2] = (float) point.getZ();
      }
   }

   public void setDecayingWorldScan(ArrayList<Point3D> pointCloud)
   {
      decayingWorldScan = new float[pointCloud.size()*3];
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3D point = pointCloud.get(i);
         decayingWorldScan[3 * i] = (float) point.getX();
         decayingWorldScan[3 * i + 1] = (float) point.getY();
         decayingWorldScan[3 * i + 2] = (float) point.getZ();
      }
   }

   public Point3D32[] getGroundQuadTreeSupport()
   {
      
      int numberOfPoints = groundQuadTreeSupport.length/3;
      
      Point3D32[] points = new Point3D32[numberOfPoints];
      for(int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(groundQuadTreeSupport[3 * i]);
         point.setY(groundQuadTreeSupport[3 * i + 1]);
         point.setZ(groundQuadTreeSupport[3 * i + 2]);
         points[i] = point;
      }
      
      return points;
   }

   public Point3D32[] getDecayingWorldScan()
   {
      int numberOfPoints = decayingWorldScan.length/3;
      
      Point3D32[] points = new Point3D32[numberOfPoints];
      for(int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(decayingWorldScan[3 * i]);
         point.setY(decayingWorldScan[3 * i + 1]);
         point.setZ(decayingWorldScan[3 * i + 2]);
         points[i] = point;
      }
      
      return points;
   }
   
   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      boolean ret = timestamp == other.timestamp;
      for (int i = 0; i < groundQuadTreeSupport.length; i++)
      {
         ret &= groundQuadTreeSupport[i] == other.groundQuadTreeSupport[i];
      }
      for (int i = 0; i < decayingWorldScan.length; i++)
      {
         ret &= decayingWorldScan[i] == other.decayingWorldScan[i];
      }
      ret &= defaultGroundHeight == other.defaultGroundHeight;
      
      return ret;
   }
   
   @Override
   public String toString()
   {
      String ret;

      try
      {
         ret = "PointCloudWorldPacket [timestamp=" + timestamp + ", groundQuadTreeSupport=" + groundQuadTreeSupport.length/3 + " points, decayingWorldScan="
               + decayingWorldScan.length/3 + " points, defaultGroundHeight=" + defaultGroundHeight + "]";
      }
      catch (NullPointerException e)
      {
         ret = getClass().getSimpleName();
      }

      return ret;
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
