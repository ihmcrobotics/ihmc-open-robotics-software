package us.ihmc.humanoidRobotics.communication.packets.sensing;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class LocalizationPointMapPacket extends Packet<LocalizationPointMapPacket>
{
   public long timestamp;
   public TFloatArrayList localizationPointMap = new TFloatArrayList();

   public LocalizationPointMapPacket()
   {
      setDestination(PacketDestination.UI);
   }

   @Override
   public void set(LocalizationPointMapPacket other)
   {
      timestamp = other.timestamp;
      MessageTools.copyData(other.localizationPointMap, localizationPointMap);
      setPacketInformation(other);
   }

   public void setLocalizationPointMap(Point3DReadOnly[] pointCloud)
   {
      localizationPointMap.reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         localizationPointMap.add((float) point.getX());
         localizationPointMap.add((float) point.getY());
         localizationPointMap.add((float) point.getZ());
      }
   }

   public Point3D32[] getPointMap()
   {

      int numberOfPoints = localizationPointMap.size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(localizationPointMap.get(3 * i));
         point.setY(localizationPointMap.get(3 * i + 1));
         point.setZ(localizationPointMap.get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   @Override
   public boolean epsilonEquals(LocalizationPointMapPacket other, double epsilon)
   {
      if (timestamp != other.timestamp)
         return false;
      if (!MessageTools.epsilonEquals(localizationPointMap, other.localizationPointMap, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "PointCloudWorldPacket [timestamp=" + timestamp + " points, localizationPointMap=" + localizationPointMap.size() / 3 + "]";
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
