package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FlatGroundProfile extends GroundProfileFromHeightMap
{
   private final double zHeight;
   private final BoundingBox3D boundingBox;

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

      this.boundingBox = new BoundingBox3D(xMin, yMin, zHeight - 1.0, xMax, yMax, zHeight + 0.01);
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
      return zHeight;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
