package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;


public class AlternatingSlopesGroundProfile extends GroundProfileFromHeightMap
{
   private BoundingBox3D boundingBox;
   
   private static final double defaultXMin = -10.0, defaultXMax = 10.0, defaultYMin = -10.0, defaultYMax = 10.0;

   private final double[][] xSlopePairs;
   private final double[][] xzPairs;

   public AlternatingSlopesGroundProfile(double[][] xSlopePairs)
   {
      this(xSlopePairs, defaultXMin, defaultXMax, defaultYMin, defaultYMax);
   }

   public AlternatingSlopesGroundProfile(double[][] xSlopePairs, double xMin, double xMax, double yMin, double yMax)
   {
      this.xSlopePairs = xSlopePairs;
      boundingBox = new BoundingBox3D(new Point3D(xMin, yMin, Double.NEGATIVE_INFINITY), new Point3D(xMax, yMax, Double.MAX_VALUE));

      modifyXMinMaxIfNecessary();

      xzPairs = createXZPairsFromXSlopePairs(xSlopePairs);
      verifyXOrdering();
      
   }

   private void modifyXMinMaxIfNecessary()
   {
      double xMin = boundingBox.getMinX();
      double xMax = boundingBox.getMaxX();
      
      boolean changed = false;
      
      if (xMin > xSlopePairs[0][0] - 1e-7)
      {
         xMin = xSlopePairs[0][0] - 1e-7;
         changed = true;
      }
      
      if (xMax < xSlopePairs[xSlopePairs.length - 1][0] + 1e-7)
      {
         xMax = xSlopePairs[xSlopePairs.length - 1][0] + 1e-7;
         changed = true;
      }
      
      if (changed)
      {
         boundingBox = new BoundingBox3D(xMin, boundingBox.getMinY(), boundingBox.getMinZ(), xMax, boundingBox.getMaxY(), boundingBox.getMaxZ());
      }
   }

   private void verifyXOrdering()
   {
      double x = boundingBox.getMinX();

      for (int i=0; i<xSlopePairs.length; i++)
      {
         if (x + 1e-7 > xSlopePairs[i][0])
         {
            throw new RuntimeException("Bad x ordering of points in AlternatingSlopesGroundProfile. Each point must increase!");
         }
         x = xSlopePairs[i][0];
      }

      if (x > boundingBox.getMaxX())
      {
         throw new RuntimeException("Bad x ordering of points in AlternatingSlopesGroundProfile. Last Point is greater than xMax!");
      }
   }

   public double[][] getXZPairs()
   {
      return xzPairs;
   }

   private double[][] createXZPairsFromXSlopePairs(double[][] xSlopePairs)
   {
      int numPairs = xSlopePairs.length + 2;

      double[][] ret = new double[numPairs][2];

      double x = 0.0;
      double z = 0.0;

      double previousX = boundingBox.getMinX();
      double previousSlope = 0.0;

      ret[0][0] = boundingBox.getMinX();
      ret[0][1] = 0.0;

      for (int i = 0; i < numPairs - 2; i++)
      {
         x = xSlopePairs[i][0];

         double run = x - previousX;
         double rise = previousSlope * run;

         z = z + rise;

         ret[i + 1][0] = x;
         ret[i + 1][1] = z;

         previousSlope = xSlopePairs[i][1];
         previousX = x;
      }

      ret[numPairs - 1][0] = boundingBox.getMaxX();

      double run = boundingBox.getMaxX() - previousX;
      double rise = previousSlope * run;

      z = z + rise;
      ret[numPairs - 1][1] = z;

      return ret;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      int indexOnLeft = findIndexOnLeft(xzPairs, x);

      if (indexOnLeft < 0)
         return 0.0;
      if (indexOnLeft == xzPairs.length - 1)
         return xzPairs[indexOnLeft][1];

      double[] xzOnLeft = xzPairs[indexOnLeft];
      double[] xzOnRight = xzPairs[indexOnLeft + 1];

      double xOnLeft = xzOnLeft[0];
      double xOnRight = xzOnRight[0];

      double zOnLeft = xzOnLeft[1];
      double zOnRight = xzOnRight[1];

      double ret = zOnLeft + ((x - xOnLeft) / (xOnRight - xOnLeft)) * (zOnRight - zOnLeft);

      return ret;
   }

   private static int findIndexOnLeft(double[][] pairs, double x)
   {
      for (int i = 0; i < pairs.length; i++)
      {
         double[] xzPair = pairs[i];

         if (x < xzPair[0])
            return i - 1;
      }

      return pairs.length - 1;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      int indexOnLeft = findIndexOnLeft(xSlopePairs, x);

      if (indexOnLeft < 0)
      {
         normal.set(0.0, 0.0, 1.0);
      }

      else if (indexOnLeft == xzPairs.length - 1)
      {
         normal.set(0.0, 0.0, 1.0);
      }

      else
      {
         double slope = xSlopePairs[indexOnLeft][1];

         normal.setX(-slope);
         normal.setY(0.0);
         normal.setZ(1.0);

         normal.normalize();
      }
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
