package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class LidarScanMessage extends Packet<LidarScanMessage>
{
   public long robotTimestamp;
   public Point3D32 lidarPosition = new Point3D32();
   public Quaternion32 lidarOrientation = new Quaternion32();
   public TFloatArrayList scan = new TFloatArrayList();

   public LidarScanMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(LidarScanMessage other)
   {
      robotTimestamp = other.robotTimestamp;
      lidarPosition = new Point3D32(other.lidarPosition);
      lidarOrientation = new Quaternion32(other.lidarOrientation);
      MessageTools.copyData(other.scan, scan);
      setPacketInformation(other);
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public void setLidarPosition(Point3DReadOnly lidarPosition)
   {
      this.lidarPosition.set(lidarPosition);
   }

   public void setLidarOrientation(QuaternionReadOnly lidarOrientation)
   {
      this.lidarOrientation.set(lidarOrientation);
   }

   public Point3D32 getLidarPosition()
   {
      return lidarPosition;
   }

   public Quaternion32 getLidarOrientation()
   {
      return lidarOrientation;
   }

   @Override
   public boolean epsilonEquals(LidarScanMessage other, double epsilon)
   {
      if (!lidarPosition.epsilonEquals(other.lidarPosition, (float) epsilon))
         return false;
      if (!lidarOrientation.epsilonEquals(other.lidarOrientation, (float) epsilon))
         return false;
      if (!MessageTools.epsilonEquals(scan, other.scan, epsilon))
         return false;
      return true;
   }
}
