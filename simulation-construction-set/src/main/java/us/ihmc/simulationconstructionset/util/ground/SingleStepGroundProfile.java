package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Vector3D;


public class SingleStepGroundProfile extends GroundProfileFromHeightMap
{
   private final BoundingBox3D boundingBox;
   private final double groundXStep, groundZStep;

   public SingleStepGroundProfile(double xMin, double xMax, double yMin, double yMax, double groundXStep, double groundZStep)
   {
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Math.max(0.0, groundZStep) + 0.01;

      boundingBox = new BoundingBox3D(xMin, yMin, zMin, xMax, yMax, zMax);

      this.groundXStep = groundXStep;
      this.groundZStep = groundZStep;
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
      if (x < groundXStep)
         return 0.0;
      else
         return groundZStep;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
