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

   private final Point3d tempPoint3dForCheckInside = new Point3d();

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
      boolean isPointInside = planarRegion.isPointInside(tempPoint3dForCheckInside, 1e-10);

      if(isPointInside)
      {
         intersectionToPack.set(tempPoint3dForCheckInside);
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
