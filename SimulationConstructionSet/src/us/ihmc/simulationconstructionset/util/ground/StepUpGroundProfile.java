package us.ihmc.simulationconstructionset.util.ground;


import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;


public class StepUpGroundProfile extends GroundProfileFromHeightMap
{
   private final BoundingBox3d boundingBox;

   private final double groundXStep;
   private final double groundZStep;

   public StepUpGroundProfile(double groundXStep, double groundZStep)
   {
      double xMin = -1.0;
      double xMax = 4.0;
      double yMin = -1.0;
      double yMax = 1.0;

      this.groundXStep = groundXStep;
      this.groundZStep = groundZStep;
      
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   @Override
   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersection, Vector3D normal)
   {
      closestIntersectionTo(x, y, z, intersection);
      surfaceNormalAt(x, y, z, normal);

   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersection)
   {
      intersection.set(x, y, heightAt(x, y, z));
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
      if (x > groundXStep)
         return groundZStep;

      return 0.0;
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
