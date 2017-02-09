package us.ihmc.simulationconstructionset.util.ground;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;


public class RandomRockyGroundProfile extends GroundProfileFromHeightMap
{
   private double resolution;

   private double startRocksAtX;
   private int fieldLength;

   private int minRockRadius, maxRockRadius;
   private int minRockLength, maxRockLength, minRockWidth, maxRockWidth;

   private float minRockHeight, maxRockHeight;


   private final BoundingBox3d boundingBox;


   private float[][] terrainMap;


   Random rand = new Random(1000);

   public RandomRockyGroundProfile()
   {
      this(20);
   }

   public RandomRockyGroundProfile(double fieldLength)
   {
      this(fieldLength, 6000);
   }

   public RandomRockyGroundProfile(double fieldLength, int numberOfRocks)
   {
      this(fieldLength, numberOfRocks, 0.001, 0.02, -fieldLength / 2.0);
   }

   public RandomRockyGroundProfile(double fieldLength, int numberOfRocks, double minRockHeight, double maxRockHeight, double startRocksAtX)
   {
      this(fieldLength, numberOfRocks, minRockHeight, maxRockHeight, 0.01, 0.1, 0.05, 0.1, 0.05, 0.1);
      this.startRocksAtX = startRocksAtX;
   }

   public RandomRockyGroundProfile(double fieldLength, int numberOfRocks, double minRockHeight, double maxRockHeight, double minRockRadius,
                                   double maxRockRadius, double minRockLength, double maxRockLength, double minRockWidth, double maxRockWidth)
   {
      this(fieldLength, numberOfRocks, minRockHeight, maxRockHeight, minRockRadius, maxRockRadius, minRockLength, maxRockLength, minRockWidth, maxRockWidth,
           0.01);
   }


   public RandomRockyGroundProfile(double fieldLength, int numberOfRocks, double minRockHeight, double maxRockHeight, double minRockRadius,
                                   double maxRockRadius, double minRockLength, double maxRockLength, double minRockWidth, double maxRockWidth,
                                   double resolution)
   {
      this.resolution = resolution;
      this.fieldLength = (int) (fieldLength / resolution);

      this.minRockHeight = (float) minRockHeight;
      this.maxRockHeight = (float) maxRockHeight;

      this.minRockRadius = (int) (minRockRadius / resolution);
      this.maxRockRadius = (int) (maxRockRadius / resolution);

      this.minRockLength = (int) (minRockLength / resolution);
      this.maxRockLength = (int) (maxRockLength / resolution);
      this.minRockWidth = (int) (minRockWidth / resolution);
      this.maxRockWidth = (int) (maxRockWidth / resolution);

      double xMin = -(this.fieldLength) * resolution / 2.0;
      double xMax = (this.fieldLength) * resolution / 2.0;
      double yMin = -(this.fieldLength) * resolution / 2.0;
      double yMax = (this.fieldLength) * resolution / 2.0;

      this.boundingBox = new BoundingBox3d(xMin, yMin, Double.NEGATIVE_INFINITY, xMax, yMax, Double.POSITIVE_INFINITY);
      
      terrainMap = new float[this.fieldLength][this.fieldLength];

      for (int i = 0; i < numberOfRocks; i++)
      {
         float height = rand.nextFloat() * (this.maxRockHeight - this.minRockHeight) + this.minRockHeight;
         int xPos = rand.nextInt(this.fieldLength);
         int yPos = rand.nextInt(this.fieldLength);

         // Chose between cylinder and rectangular rocks

         if (rand.nextBoolean())
         {
            // Cylinder
            int radius = rand.nextInt(this.maxRockRadius - this.minRockRadius) + this.minRockRadius;


            for (int x = xPos - radius; x <= xPos + radius; x++)
            {
               if ((x >= this.fieldLength) || (x < 0))
                  continue;

               for (int y = yPos - radius; y <= yPos + radius; y++)
               {
                  if ((y >= this.fieldLength) || (y < 0))
                     continue;

                  if (((x-xPos) * (x-xPos) + (y-yPos) * (y-yPos)) < radius * radius)
                  {
                     terrainMap[x][y] = height;
                  }

               }
            }


         }
         else
         {
            // Rectangular
            int xSize = rand.nextInt(this.maxRockLength - this.minRockLength) + this.minRockLength;
            int ySize = rand.nextInt(this.maxRockWidth - this.minRockWidth) + this.minRockWidth;

            for (int x = xPos - xSize / 2; x <= xPos + xSize / 2; x++)
            {
               if ((x >= this.fieldLength) || (x < 0))
                  continue;

               for (int y = yPos - ySize / 2; y <= yPos + ySize / 2; y++)
               {
                  if ((y >= this.fieldLength) || (y < 0))
                     continue;
                  terrainMap[x][y] = height;

               }
            }
         }

      }
   }

   private boolean withinBounds(int x, int y)
   {
      return ((x > 0) && (x < fieldLength) && (y > 0) && (y < fieldLength));
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }
   
   @Override
   public double heightAt(double x_world, double y_world, double z_world)
   {
      int xPos1 = (int) Math.floor((x_world - boundingBox.getXMin()) / resolution);
      int yPos1 = (int) Math.floor((y_world - boundingBox.getYMin()) / resolution);
      int xPos2 = (int) Math.ceil((x_world - boundingBox.getXMin()) / resolution);
      int yPos2 = (int) Math.ceil((y_world - boundingBox.getYMin()) / resolution);

      if (!withinBounds(xPos1, yPos1) ||!withinBounds(xPos2, yPos2))
         return 0.0;

      if (x_world < startRocksAtX)
         return 0.0;

      return (terrainMap[xPos1][yPos1] + terrainMap[xPos2][yPos2]) / 2.0;
   }


   public void surfaceNormalAt(double x_world, double y_world, double z_world, Vector3d normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);

      int xPos1 = (int) Math.floor((x_world - boundingBox.getXMin()) / resolution);
      int yPos1 = (int) Math.floor((y_world - boundingBox.getYMin()) / resolution);
      int xPos2 = (int) Math.ceil((x_world - boundingBox.getXMin()) / resolution);
      int yPos2 = (int) Math.ceil((y_world - boundingBox.getYMin()) / resolution);

      if (!withinBounds(xPos1, yPos1) ||!withinBounds(xPos2, yPos2))
         return;


      double h1 = terrainMap[xPos1][yPos1];
      double h2 = terrainMap[xPos2][yPos2];

      Point3d p1 = null;
      Point3d p2 = null;

      if (h1 > h2)
      {
         p1 = new Point3d(xPos1, yPos1, h1);
         p2 = new Point3d(xPos2, yPos2, h2);
      }
      else
      {
         p1 = new Point3d(xPos1, yPos1, h2);
         p2 = new Point3d(xPos2, yPos2, h1);
      }

      Vector3d v1 = new Vector3d();
      v1.sub(p1, p2);

      // Create a vector point away from p1

      Vector3d v2 = new Vector3d(-v1.getY(), v1.getX(), 0.0);

      normal.cross(v1, v2);
      normal.normalize();

      if (normal.getZ() < 0)
         System.out.println("The z value of the normal force should be positive");

   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
