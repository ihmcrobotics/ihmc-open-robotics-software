package us.ihmc.communication.packets;

import java.util.List;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;

public class LidarScanMessage extends Packet<LidarScanMessage>
{
   public long robotTimestamp;
   public Point3D32 lidarPosition;
   public Quaternion32 lidarOrientation;
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

   public void setLidarPosition(Point3D32 lidarPosition)
   {
      this.lidarPosition = lidarPosition;
   }

   public void setLidarPosition(Point3D lidarPosition)
   {
      this.lidarPosition = new Point3D32(lidarPosition);
   }

   public void setLidarOrientation(Quaternion32 lidarOrientation)
   {
      this.lidarOrientation = lidarOrientation;
   }

   public void setLidarOrientation(Quaternion lidarOrientation)
   {
      this.lidarOrientation = new Quaternion32(lidarOrientation);
   }

   public void setScan(float[] scan)
   {
      this.scan.reset();
      this.scan.add(scan);
   }

   public void setScan(Point3DReadOnly[] scan)
   {
      this.scan.reset();

      for (Point3DReadOnly scanPoint : scan)
      {
         this.scan.add((float) scanPoint.getX());
         this.scan.add((float) scanPoint.getY());
         this.scan.add((float) scanPoint.getZ());
      }
   }

   public Point3D32 getLidarPosition()
   {
      return lidarPosition;
   }

   public void getLidarPosition(Point3D positionToPack)
   {
      positionToPack.set(lidarPosition);
   }

   public void getLidarPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition);
   }
   
   public Quaternion32 getLidarOrientation()
   {
      return lidarOrientation;
   }

   public void getLidarOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(lidarOrientation);
   }

   public void getLidarOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarOrientation);
   }

   public void getLidarPose(FramePose3D poseToPack)
   {
      poseToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition, lidarOrientation);
   }

   public int getNumberOfScanPoints()
   {
      return scan.size() / 3;
   }

   public Point3D32 getScanPoint3f(int index)
   {
      Point3D32 scanPoint = new Point3D32();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, Point3DBasics scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(scan.get(index++));
      scanPointToPack.setY(scan.get(index++));
      scanPointToPack.setZ(scan.get(index++));
   }

   public Point3D getScanPoint3d(int index)
   {
      Point3D scanPoint = new Point3D();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public FramePoint3D getScanFramePoint(int index)
   {
      FramePoint3D scanPoint = new FramePoint3D();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, FramePoint3D scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setToZero(ReferenceFrame.getWorldFrame());
      getScanPoint(index, (Point3DBasics) scanPointToPack);
   }

   public Point3D32[] getScanPoint3fs()
   {
      Point3D32[] scanPoints = new Point3D32[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3f(index);
      return scanPoints;
   }

   public Point3D[] getScanPoint3ds()
   {
      Point3D[] scanPoints = new Point3D[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3d(index);
      return scanPoints;
   }

   public FramePoint3D[] getScanFramePoints()
   {
      FramePoint3D[] scanPoints = new FramePoint3D[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanFramePoint(index);
      return scanPoints;
   }

   public void getScanPoint3fs(List<Point3D32> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3f(index));
   }

   public void getScanPoint3ds(List<Point3D> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3d(index));
   }

   public void getScanFramePoints(List<FramePoint3D> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanFramePoint(index));
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
