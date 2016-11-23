package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.PlanarRegion;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final PlanarRegion planarRegion;
   private final Graphics3DObject linkGraphics;

   public PlanarRegionTerrainObject(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;
      this.linkGraphics = setupLinkGraphics();
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      double planeZGivenXY = planarRegion.getPlaneZGivenXY(x, y);
      if (planeZGivenXY >= z)
      {
         return planeZGivenXY;
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      planarRegion.getNormal(normalToPack);
      return this.heightAt(x, y, z);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return planarRegion.getBoundingBox3dInWorld();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return this.linkGraphics;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if(planarRegion.isPointInsideByProjectionOntoXYPlane(x, y))
      {
         double planeZGivenXY = planarRegion.getPlaneZGivenXY(x, y);
         return planeZGivenXY > z - 1e-12;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      // We will recycle intersectionToPack here so we don't have to make an allocation.
      double oldX, oldY, oldZ;

      // Store current values of intersectionToPack
      oldX = intersectionToPack.x;
      oldY = intersectionToPack.y;
      oldZ = intersectionToPack.z;

      // Set intersection to pack with the values to test so that we can use it with planarRegion's isPointInside() method call
      intersectionToPack.x = x;
      intersectionToPack.y = y;
      intersectionToPack.z = z;

      // Check if the point is inside, modulo some small epislon
      boolean isPointInside = planarRegion.isPointInside(intersectionToPack, 1e-3);

      if(!isPointInside)
      {
         // If the point is not inside, switch intersectionToPack back to its original values.
         // Just in case the caller of this method was using those old values for something in
         // event of failure
         intersectionToPack.x = oldX;
         intersectionToPack.y = oldY;
         intersectionToPack.z = oldZ;
      }
      else
      {
         // If the point is inside, update the normal to pack.
         planarRegion.getNormal(normalToPack);
      }

      return isPointInside;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   private Graphics3DObject setupLinkGraphics()
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addPlanarRegion(planarRegion, YoAppearance.Gray());
      return graphics3DObject;
   }
}
