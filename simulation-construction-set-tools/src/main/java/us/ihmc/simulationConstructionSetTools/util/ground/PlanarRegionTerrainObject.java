package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 *
 * Note: The bahvior of contacts occuring at the intersection of multiple planar regions (e.g.,
 *       when stepping with partial footholds) might not be as accurate. In these cases, the
 *       @p thickness by which the convex polytopes are extruded ends up affecting the response
 *       of the physics engine at the time of collision.
 */
public class PlanarRegionTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final PlanarRegion planarRegion;
   private final double allowablePenetrationThickness;
   private final Graphics3DObject linkGraphics;
   private final AppearanceDefinition appearance;

   private final ArrayList<ConvexPolytope3D> planarCollisionShape = new ArrayList<>();

   private final Point3D tempPoint3dForCheckInside = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final Vector3D terrainNormal = new Vector3D();

   public PlanarRegionTerrainObject(PlanarRegion planarRegion, double allowablePenetrationThickness)
   {
      this(planarRegion, allowablePenetrationThickness, YoAppearance.Gray());
   }

   public PlanarRegionTerrainObject(PlanarRegion planarRegion, double allowablePenetrationThickness, AppearanceDefinition appearance)
   {
      this.planarRegion = planarRegion;
      this.allowablePenetrationThickness = allowablePenetrationThickness;
      this.appearance = appearance;
      this.linkGraphics = setupLinkGraphics();

      createCollisionShapeForPlanarRegion(planarRegion, allowablePenetrationThickness);

      this.planarRegion.setBoundingBoxEpsilon(allowablePenetrationThickness);

      planarRegion.getNormal(terrainNormal);

      if(terrainNormal.getZ() < 0.0)
         terrainNormal.negate();
   }

   private void createCollisionShapeForPlanarRegion(PlanarRegion planarRegion, double thickness)
   {
      ConvexPolytope3D extrudedPolygon = new ConvexPolytope3D();
      Point3D tmpVertex = new Point3D();
      Point2DReadOnly tmpVertex2D;

      List<ConvexPolygon2D> planarPolygons = planarRegion.getConvexPolygons();
      int numberOfVertices;
      for (ConvexPolygon2D planarPolygon : planarPolygons)
      {
         extrudedPolygon.clear();
         numberOfVertices = planarPolygon.getNumberOfVertices();
         for (int vertex = 0; vertex < numberOfVertices; vertex++)
         {
            tmpVertex2D = planarPolygon.getVertex(vertex);
            tmpVertex.set(tmpVertex2D.getX(), tmpVertex2D.getY(), 0.0);
            extrudedPolygon.addVertex(tmpVertex);
            tmpVertex.setZ(-thickness);
            extrudedPolygon.addVertex(tmpVertex);
         }
         extrudedPolygon.applyTransform(planarRegion.getTransformToWorld());
         planarCollisionShape.add(extrudedPolygon.copy());
      }
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      if (planarRegion.isPointInsideByProjectionOntoXYPlane(x, y))
      {
         return planarRegion.getPlaneZGivenXY(x, y);
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      if (planarRegion.isPointInsideByProjectionOntoXYPlane(x, y))
      {
         if (normalToPack != null)
         {
            normalToPack.set(terrainNormal);
         }

         return planarRegion.getPlaneZGivenXY(x, y);
      }
      else
      {
         normalToPack.set(0.0, 0.0, 1.0);
         return 0.0;
      }
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return planarRegion.getBoundingBox3dInWorld();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return this.linkGraphics;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean isClose(double x, double y, double z)
   {
      return planarRegion.getBoundingBox3dInWorld().isXYInsideInclusive(x, y);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      tempPoint3dForCheckInside.setX(x);
      tempPoint3dForCheckInside.setY(y);
      tempPoint3dForCheckInside.setZ(z);

      boolean isPointInside;
      if(planarRegion.getNormal().getZ() > 0.0)
      {
         isPointInside = planarRegion.isPointOnOrSlightlyBelow(tempPoint3dForCheckInside, allowablePenetrationThickness);
      }
      else
      {
         isPointInside = planarRegion.isPointOnOrSlightlyAbove(tempPoint3dForCheckInside, allowablePenetrationThickness);
      }

      if (isPointInside)
      {
         if (intersectionToPack != null)
         {
            intersectionToPack.set(tempPoint3dForCheckInside);
         }

         if (normalToPack != null)
         {
            normalToPack.set(terrainNormal);
         }
      }

      return isPointInside;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return planarCollisionShape;
   }

   private Graphics3DObject setupLinkGraphics()
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      double thickness = Math.max(allowablePenetrationThickness, 1e-4);
      Graphics3DObjectTools.addPlanarRegion(graphics3DObject, planarRegion, thickness, appearance);
      return graphics3DObject;
   }
}
