package us.ihmc.darpaRoboticsChallenge.sensorProcessing.sensorData;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class DRCLidarScanDeleted
{
   public float[] distances;
   public int[] indexes;
   public Point3d origin;
   public Quat4d rotation;
   //public boolean isSCS;

   
   public DRCLidarScanDeleted()
   {
      
   }
   
   public DRCLidarScanDeleted(boolean isSCS, Point3d origin, Quat4d rotation, int[] indexes, float[] distances)
   {
      //this.isSCS = isSCS;
      this.origin = origin;
      this.rotation = rotation;
      this.indexes = indexes;
      this.distances = distances;
   }

   public float[] getDistances()
   {
      return distances;
   }

   public int[] getIndexes()
   {
      return indexes;
   }

   public Point3d getOrigin()
   {
      return origin;
   }

   public Quat4d getRotation()
   {
      return rotation;
   }

   /*
   public boolean getIsSCS()
   {
      return isSCS;
   }*/

   public void setDistances(float[] distances)
   {
      this.distances = distances;
   }

   public void setIndexes(int[] indexes)
   {
      this.indexes = indexes;
   }

   public void setOrigin(Point3d origin)
   {
      this.origin = origin;
   }

   public void setRotation(Quat4d rotation)
   {
      this.rotation = rotation;
   }

   /*
   public void setIsSCS(boolean isSCS)
   {
      this.isSCS = isSCS;
   }*/

   


}
