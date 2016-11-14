package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final PlanarRegion planarRegion;
   private final BoundingBox3d boundingBox3d;
   private final Graphics3DObject linkGraphics;

   public PlanarRegionTerrainObject(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;
      this.boundingBox3d = setupBoundingBox3d();
      this.linkGraphics = new Graphics3DObject();
      linkGraphics.addPlanarRegion(planarRegion);
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
      return boundingBox3d;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return this.linkGraphics;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return false;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      return false;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return null;
   }

   private BoundingBox3d setupBoundingBox3d()
   {
      double xMin, xMax, yMin, yMax, zMin, zMax;
      xMin = xMax = yMin = yMax = zMin = zMax = 0.0;

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(planarRegionTransformToWorld);

      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2d convexPolygonInWorld = planarRegion.getConvexPolygon(i).applyTransformCopy(planarRegionTransformToWorld);

         for (int j = 0; j < convexPolygonInWorld.getNumberOfVertices(); j++)
         {
            Point2d vertex = convexPolygonInWorld.getVertex(j);
            double planeZGivenXY = planarRegion.getPlaneZGivenXY(vertex.x, vertex.y);

            if (planeZGivenXY > zMax)
            {
               zMax = planeZGivenXY;
            }

            if (planeZGivenXY < zMin)
            {
               zMin = planeZGivenXY;
            }
         }

         Point2d maxPoint2d = convexPolygonInWorld.getBoundingBox().getMaxPoint();
         Point2d minPoint2d = convexPolygonInWorld.getBoundingBox().getMinPoint();

         if (minPoint2d.x < xMin)
         {
            xMin = minPoint2d.x;
         }

         if (minPoint2d.y < yMin)
         {
            yMin = minPoint2d.y;
         }

         if (maxPoint2d.x > xMax)
         {
            xMax = maxPoint2d.x;
         }

         if (maxPoint2d.y > yMax)
         {
            yMax = maxPoint2d.y;
         }

      }

      return new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }
}
