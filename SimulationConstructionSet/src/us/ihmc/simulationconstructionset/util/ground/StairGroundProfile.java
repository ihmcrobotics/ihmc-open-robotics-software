package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;


public class StairGroundProfile extends GroundProfileFromHeightMap
{
   private final BoundingBox3d boundingBox;

   private final double groundXStep, groundZStep;
   private final double startStairsAtX;

   public StairGroundProfile(double groundXStep, double groundZStep)
   {
      double xMin = -1.0;
      double xMax = 4.0;
      double yMin = -1.0;
      double yMax = 1.0;

      this.startStairsAtX = 0.0;

      this.groundXStep = groundXStep;
      this.groundZStep = groundZStep;

      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }


   public StairGroundProfile(double groundXStep, double groundZStep, double xMin, double xMax, double yMin, double yMax, double startStairsAtX)
   {
      this.startStairsAtX = startStairsAtX;

      this.groundXStep = groundXStep;
      this.groundZStep = groundZStep;

      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);

      return height;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      int stairNumber = (int) Math.ceil((x - startStairsAtX / groundXStep));    // the ceil ensures that the fist step is always at x = 0.0, which simplifies initial robot setup

      if ((x < startStairsAtX) || (y < boundingBox.getYMin()) || (y > boundingBox.getYMax()))
         return 0.0;
      else
         return stairNumber * groundZStep;

   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
