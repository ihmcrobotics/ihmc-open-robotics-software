package us.ihmc.communication.packets.sensing;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.random.RandomTools;

public abstract class AbstractPointCloudPacket extends Packet<AbstractPointCloudPacket>
{
   public Point3d origin;
   public float[] flatPoints;
   public long timeStamp;
   
   public AbstractPointCloudPacket()
   {
   }

   public AbstractPointCloudPacket(Random random) 
   {
	   origin = RandomTools.generateRandomPoint(random, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
	   timeStamp = random.nextLong();
	   
	   int size = Math.abs(random.nextInt(1000000));
	   flatPoints = new float[size];
	   for(int i = 0; i < flatPoints.length; i++)
	   {
		   flatPoints[i] = random.nextFloat();
	   }
   }
   
   public AbstractPointCloudPacket(Point3d origin, Point3d[] pointCloud, long timeStamp)
   {
      this.origin = new Point3d(origin);
      this.timeStamp = timeStamp;
      flatPoints = new float[pointCloud.length * 3];
      for (int i = 0; i < pointCloud.length; i++)
      {
         flatPoints[3 * i] = (float) pointCloud[i].getX();
         flatPoints[3 * i + 1] = (float) pointCloud[i].getY();
         flatPoints[3 * i + 2] = (float) pointCloud[i].getZ();
      }
   }

   public AbstractPointCloudPacket(Point3d origin, ArrayList<Point3d> pointCloud, long timeStamp)
   {
      this.origin = origin;
      this.timeStamp = timeStamp;
      flatPoints = new float[pointCloud.size() * 3];
      Point3d point;
      for (int i = 0; i < pointCloud.size(); i++)
      {
         point = pointCloud.get(i);
         flatPoints[3 * i] = (float) point.getX();
         flatPoints[3 * i + 1] = (float) point.getY();
         flatPoints[3 * i + 2] = (float) point.getZ();
      }
   }

   public float[] getFlattenedPoints()
   {
      return flatPoints;
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public Point3f[] getPoints3f()
   {
      Point3f[] points = new Point3f[flatPoints.length / 3];

      for (int i = 0; i < points.length; i++)
      {
         points[i] = new Point3f(flatPoints[3 * i], flatPoints[3 * i + 1], flatPoints[3 * i + 2]);
      }
      return points;
   }

   public Point3d[] getPoints()
   {
      Point3d[] points = new Point3d[flatPoints.length / 3];

      for (int i = 0; i < points.length; i++)
      {
         points[i] = new Point3d(flatPoints[3 * i], flatPoints[3 * i + 1], flatPoints[3 * i + 2]);
      }
      return points;
   }

   public int getNumberOfFlattenedPoints()
   {
      return flatPoints.length;
   }

   public int getNumberOfPoints()
   {
      return flatPoints.length / 3;
   }

   public boolean epsilonEquals(AbstractPointCloudPacket other, double epsilon)
   {
      if (timeStamp != other.timeStamp)
         return false;

      if (flatPoints.length != other.flatPoints.length)
         return false;

      for (int i = 0; i < flatPoints.length; i++)
      {
         if (Math.abs(other.flatPoints[i] - flatPoints[i]) > epsilon)
         {
            return false;
         }
      }
      return true;
   }
}
