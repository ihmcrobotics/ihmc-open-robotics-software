package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;

public class FlatGroundProfile extends GroundProfileFromHeightMap
{
   private final double zHeight;
   private final BoundingBox3d boundingBox;

   public FlatGroundProfile()
   {
      this(0.0);
   }

   public FlatGroundProfile(double zHeight)
   {
      this(-500.0, 500.0, -500.0, 500.0, zHeight);
   }

   public FlatGroundProfile(double xMin, double xMax, double yMin, double yMax)
   {
      this(xMin, xMax, yMin, yMax, 0.0);
   }

   public FlatGroundProfile(double xMin, double xMax, double yMin, double yMax, double zHeight)
   {
      this.zHeight = zHeight;

      this.boundingBox = new BoundingBox3d(xMin, yMin, zHeight - 1.0, xMax, yMax, zHeight + 0.01);
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
      return zHeight;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
