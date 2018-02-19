package us.ihmc.humanoidRobotics.communication.packets.sensing;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

@HighBandwidthPacket
public class PointCloudWorldPacket extends Packet<PointCloudWorldPacket>
{
   public long timestamp;

   public TFloatArrayList groundQuadTreeSupport = new TFloatArrayList();

   // Code is duplicated, probably gets replaced with locality hash
   public TFloatArrayList decayingWorldScan = new TFloatArrayList();

   public float defaultGroundHeight;

   public PointCloudWorldPacket()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDestination(PacketDestination.BROADCAST);
   }

   @Override
   public void set(PointCloudWorldPacket other)
   {
      timestamp = other.timestamp;
      MessageTools.copyData(other.groundQuadTreeSupport, groundQuadTreeSupport);
      MessageTools.copyData(other.decayingWorldScan, decayingWorldScan);
      defaultGroundHeight = other.defaultGroundHeight;
      setPacketInformation(other);
   }

   public void setGroundQuadTreeSupport(Point3D[] pointCloud)
   {
      groundQuadTreeSupport.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         groundQuadTreeSupport.add((float) point.getX());
         groundQuadTreeSupport.add((float) point.getY());
         groundQuadTreeSupport.add((float) point.getZ());
      }
   }

   public void setDecayingWorldScan(Point3D[] pointCloud)
   {
      decayingWorldScan.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         decayingWorldScan.add((float) point.getX());
         decayingWorldScan.add((float) point.getY());
         decayingWorldScan.add((float) point.getZ());
      }
   }

   public Point3D32[] getDecayingWorldScan()
   {
      int numberOfPoints = decayingWorldScan.size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(decayingWorldScan.get(3 * i));
         point.setY(decayingWorldScan.get(3 * i + 1));
         point.setZ(decayingWorldScan.get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      boolean ret = timestamp == other.timestamp;
      if (!MessageTools.epsilonEquals(groundQuadTreeSupport, other.groundQuadTreeSupport, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(decayingWorldScan, other.decayingWorldScan, epsilon))
         return false;
      ret &= defaultGroundHeight == other.defaultGroundHeight;

      return ret;
   }

   @Override
   public String toString()
   {
      String ret;

      try
      {
         ret = "PointCloudWorldPacket [timestamp=" + timestamp + ", groundQuadTreeSupport=" + groundQuadTreeSupport.size() / 3 + " points, decayingWorldScan="
               + decayingWorldScan.size() / 3 + " points, defaultGroundHeight=" + defaultGroundHeight + "]";
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
