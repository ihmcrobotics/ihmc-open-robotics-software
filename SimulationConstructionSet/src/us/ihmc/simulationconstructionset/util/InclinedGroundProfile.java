package us.ihmc.simulationconstructionset.util;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.simulationconstructionset.util.ground.GroundProfileFromHeightMap;


public class InclinedGroundProfile extends GroundProfileFromHeightMap
{
   private static final double xMinDefault = -20.0, xMaxDefault = 20.0, yMinDefault = -20.0, yMaxDefault = 20.0;
   private static final double heightOffset = -0.5;
   private static final double angleOfInclinationDefault = 0.0;

   private final BoundingBox3d boundingBox;
   private final double angleOfInclination;

   public InclinedGroundProfile()
   {
      this(angleOfInclinationDefault);
   }

   public InclinedGroundProfile(double angleOfInclination)
   {
      this(angleOfInclination, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }

   public InclinedGroundProfile(double angleOfInclination, double xMin, double xMax, double yMin, double yMax)
   {
      this.angleOfInclination = angleOfInclination;
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
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
      double height = 0.0;

      if (boundingBox.isXYInside(x, y))
         height = -x * Math.tan(angleOfInclination) + heightOffset;

      return height;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      double dzdx = 0.0;
      if (boundingBox.isXYInside(x, y))
         dzdx = -Math.tan(angleOfInclination);

      normal.setX(-dzdx);
      normal.setY(0.0);
      normal.setZ(1.0);

      normal.normalize();
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
