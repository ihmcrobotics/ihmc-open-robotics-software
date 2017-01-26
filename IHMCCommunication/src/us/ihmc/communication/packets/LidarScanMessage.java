package us.ihmc.communication.packets;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LidarScanMessage extends Packet<LidarScanMessage>
{
   public long robotTimestamp;
   public Point3f lidarPosition;
   public Quat4f lidarOrientation;
   public float[] scan;

   public LidarScanMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public LidarScanMessage(long timestamp, Point3f lidarPosition, Quat4f lidarOrientation, float[] scan)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotTimestamp = timestamp;
      this.lidarPosition = lidarPosition;
      this.lidarOrientation = lidarOrientation;
      this.scan = scan;
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public void setLidarPosition(Point3f lidarPosition)
   {
      this.lidarPosition = lidarPosition;
   }

   public void setLidarPosition(Point3d lidarPosition)
   {
      this.lidarPosition = new Point3f(lidarPosition);
   }

   public void setLidarOrientation(Quat4f lidarOrientation)
   {
      this.lidarOrientation = lidarOrientation;
   }

   public void setLidarOrientation(Quat4d lidarOrientation)
   {
      this.lidarOrientation = new Quat4f(lidarOrientation);
   }

   public void setScan(float[] scan)
   {
      this.scan = scan;
   }

   public void setScan(Point3d[] scan)
   {
      this.scan = new float[scan.length * 3];

      int index = 0;

      for (Point3d scanPoint : scan)
      {
         this.scan[index++] = (float) scanPoint.getX();
         this.scan[index++] = (float) scanPoint.getY();
         this.scan[index++] = (float) scanPoint.getZ();
      }
   }

   public void setScan(Point3f[] scan)
   {
      this.scan = new float[scan.length * 3];

      int index = 0;

      for (Point3f scanPoint : scan)
      {
         this.scan[index++] = scanPoint.getX();
         this.scan[index++] = scanPoint.getY();
         this.scan[index++] = scanPoint.getZ();
      }
   }

   public Point3f getLidarPosition()
   {
      return lidarPosition;
   }

   public void getLidarPosition(Point3d positionToPack)
   {
      positionToPack.set(lidarPosition);
   }

   public void getLidarPosition(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition);
   }
   
   public Quat4f getLidarOrientation()
   {
      return lidarOrientation;
   }

   public void getLidarOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(lidarOrientation);
   }

   public void getLidarOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarOrientation);
   }

   public void getLidarPose(FramePose poseToPack)
   {
      poseToPack.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition, lidarOrientation);
   }

   public int getNumberOfScanPoints()
   {
      return scan.length / 3;
   }

   public Point3f getScanPoint3f(int index)
   {
      Point3f scanPoint = new Point3f();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, Point3f scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public Point3d getScanPoint3d(int index)
   {
      Point3d scanPoint = new Point3d();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, Point3d scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public FramePoint getScanFramePoint(int index)
   {
      FramePoint scanPoint = new FramePoint();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, FramePoint scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setToZero(ReferenceFrame.getWorldFrame());
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public Point3f[] getScanPoint3fs()
   {
      Point3f[] scanPoints = new Point3f[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3f(index);
      return scanPoints;
   }

   public Point3d[] getScanPoint3ds()
   {
      Point3d[] scanPoints = new Point3d[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3d(index);
      return scanPoints;
   }

   public FramePoint[] getScanFramePoints()
   {
      FramePoint[] scanPoints = new FramePoint[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanFramePoint(index);
      return scanPoints;
   }

   public void getScanPoint3fs(List<Point3f> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3f(index));
   }

   public void getScanPoint3ds(List<Point3d> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3d(index));
   }

   public void getScanFramePoints(List<FramePoint> scanPointsToPack)
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
      if (scan.length != other.scan.length)
         return false;
      for (int i = 0; i < scan.length; i++)
      {
         if (!MathTools.epsilonEquals(scan[i], other.scan[i], epsilon))
            return false;
      }
      return true;
   }
}
