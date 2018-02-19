package us.ihmc.communication.packets;

import java.awt.Color;
import java.util.List;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
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
   public TFloatArrayList pointCloud = new TFloatArrayList();
   public TIntArrayList colors = new TIntArrayList();

   public StereoVisionPointCloudMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(StereoVisionPointCloudMessage other)
   {
      robotTimestamp = other.robotTimestamp;
      MessageTools.copyData(other.pointCloud, pointCloud);
      MessageTools.copyData(other.colors, colors);
      setPacketInformation(other);
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public void setPointCloudData(float[] pointCloudData, int[] colors)
   {
      this.pointCloud.reset();
      this.pointCloud.add(pointCloudData);
      this.colors.reset();
      this.colors.add(colors);

      if (pointCloudData.length != colors.length)
      {
         new RuntimeException("PointCloud incompatible with colors");
      }
   }

   public void setPointCloudData(Point3DReadOnly[] pointCloudData, Color[] colors)
   {
      this.pointCloud.reset();

      for (Point3DReadOnly scanPoint : pointCloudData)
      {
         this.pointCloud.add(scanPoint.getX32());
         this.pointCloud.add(scanPoint.getY32());
         this.pointCloud.add(scanPoint.getZ32());
      }

      this.colors.reset();;

      for (int i = 0; i < colors.length; i++)
      {
         this.colors.add(colors[i].getRGB());
      }
   }

   public int getNumberOfPointCloudPoints()
   {
      return pointCloud.size() / 3;
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
      scanPointToPack.setX(pointCloud.get(index++));
      scanPointToPack.setY(pointCloud.get(index++));
      scanPointToPack.setZ(pointCloud.get(index++));
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
      scanPointToPack.setX(pointCloud.get(index++));
      scanPointToPack.setY(pointCloud.get(index++));
      scanPointToPack.setZ(pointCloud.get(index++));
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

   public TIntArrayList getColors()
   {
      return colors;
   }

   public Color[] getAwtColors()
   {
      Color[] colors = new Color[this.colors.size()];

      for (int i = 0; i < this.colors.size(); i++)
      {
         colors[i] = new Color(this.colors.get(i));
      }

      return colors;
   }
  

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if (pointCloud.size() != other.pointCloud.size())
         return false;
      for (int i = 0; i < pointCloud.size(); i++)
      {
         if (!MathTools.epsilonEquals(pointCloud.get(i), other.pointCloud.get(i), epsilon))
            return false;
      }

      return colors.equals(other.colors);
   }
}
