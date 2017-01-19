package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class SlopedPlaneGroundProfile implements GroundProfile3D
{
   private final Vector3d surfaceNormal = new Vector3d();
   private final Point3d intersectionPoint = new Point3d();

   private final BoundingBox3d boundingBox;

   public SlopedPlaneGroundProfile(Vector3d surfaceNormal, Point3d intersectionPoint, double maxXY)
   {
      this.surfaceNormal.set(surfaceNormal);
      this.surfaceNormal.normalize();
      if (Math.abs(this.surfaceNormal.lengthSquared() - 1.0) > 1e-7)
         throw new RuntimeException("Bad Surface Normal!");

      this.intersectionPoint.set(intersectionPoint);

      double maxZ = intersectionPoint.getZ()
                    + 1.0 / Math.abs(surfaceNormal.getZ())
                      * (Math.abs(surfaceNormal.getX()) * (maxXY + Math.abs(intersectionPoint.getX()))
                         + Math.abs(surfaceNormal.getY()) * (maxXY + Math.abs(intersectionPoint.getY()))) + 0.01;

      boundingBox = new BoundingBox3d(-maxXY, -maxXY, Double.NEGATIVE_INFINITY, maxXY, maxXY, maxZ);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInside(x, y, z);
   }

   private final Vector3d intersectionToQueryVector = new Vector3d();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      normalToPack.set(surfaceNormal);

      intersectionToQueryVector.set(x, y, z);
      intersectionToQueryVector.sub(intersectionPoint);

      double dotProduct = intersectionToQueryVector.dot(surfaceNormal);

      intersectionToPack.set(x, y, z);
      intersectionToPack.scaleAdd(-dotProduct, surfaceNormal, intersectionToPack);

      return (dotProduct <= 0.0);
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      // No height map if upside down or straight vertical.
      if (surfaceNormal.getZ() < 1e-7)
         return null;

      return new HeightMapWithNormals()
      {
         @Override
         public double heightAt(double x, double y, double z)
         {
            double pz = intersectionPoint.getZ()
                        + 1.0 / surfaceNormal.getZ()
                          * (surfaceNormal.getX() * (-x + intersectionPoint.getX()) + surfaceNormal.getY() * (-y + intersectionPoint.getY()));

            return pz;
         }

         @Override
         public BoundingBox3d getBoundingBox()
         {
            return boundingBox;
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
         {
            normalToPack.set(surfaceNormal);

            return heightAt(x, y, z);
         }
      };
   }

}
