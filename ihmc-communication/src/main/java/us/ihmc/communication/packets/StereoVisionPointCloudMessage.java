package us.ihmc.communication.packets;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class StereoVisionPointCloudMessage extends Packet<StereoVisionPointCloudMessage>
{
   public long robotTimestamp;
   public float[] pointCloud;
   public int[] colors;

   public StereoVisionPointCloudMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public StereoVisionPointCloudMessage(long timestamp, float[] pointCloud, int[] colors)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotTimestamp = timestamp;
      this.pointCloud = pointCloud;
      this.colors = colors;
   }

   @Override
   public void set(StereoVisionPointCloudMessage other)
   {
      robotTimestamp = other.robotTimestamp;
      pointCloud = Arrays.copyOf(other.pointCloud, other.pointCloud.length);
      colors = Arrays.copyOf(other.colors, other.colors.length);
      setPacketInformation(other);
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public void setPointCloudData(float[] pointCloudData, int[] colors)
   {
      this.pointCloud = pointCloudData;
      this.colors = colors;

      if (pointCloudData.length != colors.length)
      {
         new RuntimeException("PointCloud incompatible with colors");
      }
   }

   public void setPointCloudData(Point3DReadOnly[] pointCloudData, Color[] colors)
   {
      this.pointCloud = new float[pointCloudData.length * 3];

      int index = 0;

      for (Point3DReadOnly scanPoint : pointCloudData)
      {
         this.pointCloud[index++] = (float) scanPoint.getX();
         this.pointCloud[index++] = (float) scanPoint.getY();
         this.pointCloud[index++] = (float) scanPoint.getZ();
      }

      this.colors = new int[colors.length];

      for (int i = 0; i < colors.length; i++)
      {
         this.colors[i] = colors[i].getRGB();
      }
   }

   public int getNumberOfPointCloudPoints()
   {
      return pointCloud.length / 3;
   }

   public Point3D32 getPoint3D32(int index)
   {
      Point3D32 scanPoint = new Point3D32();
      getPoint(index, scanPoint);
      return scanPoint;
   }

   public void getPoint(int index, Point3DBasics scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(pointCloud[index++]);
      scanPointToPack.setY(pointCloud[index++]);
      scanPointToPack.setZ(pointCloud[index++]);
   }

   public Point3D getScanPoint3d(int index)
   {
      Point3D scanPoint = new Point3D();
      getPoint(index, scanPoint);
      return scanPoint;
   }

   public FramePoint3D getFramePoint(int index)
   {
      FramePoint3D scanPoint = new FramePoint3D();
      getPoint(index, scanPoint);
      return scanPoint;
   }

   public void getPoint(int index, FramePoint3D scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setToZero(ReferenceFrame.getWorldFrame());
      scanPointToPack.setX(pointCloud[index++]);
      scanPointToPack.setY(pointCloud[index++]);
      scanPointToPack.setZ(pointCloud[index++]);
   }

   public Point3D32[] getPoint3D32s()
   {
      Point3D32[] points = new Point3D32[getNumberOfPointCloudPoints()];
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         points[index] = getPoint3D32(index);
      return points;
   }

   public Point3D[] getPoint3Ds()
   {
      Point3D[] points = new Point3D[getNumberOfPointCloudPoints()];
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         points[index] = getScanPoint3d(index);
      return points;
   }

   public FramePoint3D[] getFramePoints()
   {
      FramePoint3D[] points = new FramePoint3D[getNumberOfPointCloudPoints()];
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         points[index] = getFramePoint(index);
      return points;
   }

   public void getPoint3D32s(List<Point3D32> pointsToPack)
   {
      pointsToPack.clear();
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         pointsToPack.add(getPoint3D32(index));
   }

   public void getPoint3Ds(List<Point3D> pointsToPack)
   {
      pointsToPack.clear();
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         pointsToPack.add(getScanPoint3d(index));
   }

   public void getFramePoints(List<FramePoint3D> pointsToPack)
   {
      pointsToPack.clear();
      for (int index = 0; index < getNumberOfPointCloudPoints(); index++)
         pointsToPack.add(getFramePoint(index));
   }

   public int[] getColors()
   {
      return colors;
   }

   public Color[] getAwtColors()
   {
      Color[] colors = new Color[this.colors.length];

      for (int i = 0; i < this.colors.length; i++)
      {
         colors[i] = new Color(this.colors[i]);
      }

      return colors;
   }
  

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if (pointCloud.length != other.pointCloud.length)
         return false;
      for (int i = 0; i < pointCloud.length; i++)
      {
         if (!MathTools.epsilonEquals(pointCloud[i], other.pointCloud[i], epsilon))
            return false;
      }

      return Arrays.equals(colors, other.colors);
   }
}
