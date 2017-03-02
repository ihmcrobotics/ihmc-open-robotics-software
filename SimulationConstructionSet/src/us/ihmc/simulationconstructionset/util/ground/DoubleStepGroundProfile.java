package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class DoubleStepGroundProfile extends GroundProfileFromHeightMap
{
   private final BoundingBox3d boundingBox;

   private final double initialGroundXStep, finalGroundXStep, initialGroundZStep, finalGroundZStep;

   public DoubleStepGroundProfile(double yMin, double yMax, double initialElevationChangeX, double finalElevationChangeX, double initialElevationDifference,
                                  double finalElevationDifference)
   {
      double xMin = -1.0;
      double xMax = finalElevationChangeX + 10.0;

      initialGroundXStep = initialElevationChangeX;
      finalGroundXStep = finalElevationChangeX;
      initialGroundZStep = initialElevationDifference;
      finalGroundZStep = finalElevationDifference;

      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Math.max(0.0, Math.max(initialGroundZStep, finalGroundZStep)) + 0.01;
      boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
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
      if (x < initialGroundXStep)
         return 0.0;
      else if (x < finalGroundXStep)
         return initialGroundZStep;
      else
         return finalGroundZStep;

   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
