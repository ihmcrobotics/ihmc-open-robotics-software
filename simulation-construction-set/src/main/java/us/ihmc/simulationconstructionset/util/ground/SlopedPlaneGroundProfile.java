package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class SlopedPlaneGroundProfile implements GroundProfile3D
{
   private final Vector3D surfaceNormal = new Vector3D();
   private final Point3D intersectionPoint = new Point3D();

   private final BoundingBox3D boundingBox;

   public SlopedPlaneGroundProfile(Vector3D surfaceNormal, Point3D intersectionPoint, double maxXY)
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

      boundingBox = new BoundingBox3D(-maxXY, -maxXY, Double.NEGATIVE_INFINITY, maxXY, maxXY, maxZ);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
   }

   private final Vector3D intersectionToQueryVector = new Vector3D();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
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
         public BoundingBox3D getBoundingBox()
         {
            return boundingBox;
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
         {
            normalToPack.set(surfaceNormal);

            return heightAt(x, y, z);
         }
      };
   }

}
