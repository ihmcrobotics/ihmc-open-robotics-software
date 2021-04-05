package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.physics.Collidable;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionCollidableConverter
{
   public static List<Collidable> createCollidables(PlanarRegionsList planarRegionsList, long collisionMask, long collisionGroup)
   {
      List<Collidable> collidables = new ArrayList<>();
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         collidables.add(createCollidable(planarRegionsList.getPlanarRegion(i), collisionMask, collisionGroup));
      }

      return collidables;
   }

   public static Collidable createCollidable(PlanarRegion planarRegion, long collisionMask, long collisionGroup)
   {
      RigidBody rigidBody = null;
      FrameConvexPolytope3D regionShape = new FrameConvexPolytope3D();

      double extrusionDistance = 1e-2;
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();

      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point3D vertex = new Point3D(convexHull.getVertex(i));
         Point3D extrudedVertex = new Point3D(convexHull.getVertex(i));
         extrudedVertex.setZ(-extrusionDistance);

         planarRegion.getTransformToWorld().transform(vertex);
         planarRegion.getTransformToWorld().transform(extrudedVertex);

         regionShape.addVertex(vertex);
         regionShape.addVertex(extrudedVertex);
      }

      return new Collidable(rigidBody, collisionMask, collisionGroup, regionShape);
   }
}
