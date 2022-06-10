package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.physics.RobotCollisionModel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlanarRegionContactDetector extends FlatGroundContactDetector
{
   private PlanarRegionsList planarRegionsList = null;
   private final HashMap<PlanarRegion, ConvexPolytope3DReadOnly> contactableVolumeMap = new HashMap<>();

   public PlanarRegionContactDetector(RigidBodyBasics rootBody,
                                      RobotCollisionModel collisionModel,
                                      YoGraphicsListRegistry graphicsListRegistry,
                                      List<String> collidableRigidBodies)
   {
      super(rootBody, collisionModel, graphicsListRegistry, collidableRigidBodies);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      contactableVolumeMap.clear();

      if (planarRegionsList == null)
         return;

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = planarRegionsList.getPlanarRegion(i);
         ConvexPolytope3D contactablePolytope = new ConvexPolytope3D();

         ConvexPolygon2D convexHull = region.getConvexHull();
         for (double sign : new double[]{-1.0, 1.0})
         {
            for (int j = 0; j < convexHull.getNumberOfVertices(); j++)
            {
               Point3D vertex = new Point3D(convexHull.getVertex(j).getX(), convexHull.getVertex(j).getY(), sign * contactThreshold.getValue());
               vertex.applyTransform(region.getTransformToWorld());
               contactablePolytope.addVertex(vertex);
            }
         }
         contactableVolumeMap.put(region, contactablePolytope);
      }
   }

   @Override
   public void update(double time)
   {
      if (planarRegionsList == null)
      {
         super.update(time);
         return;
      }

      clearContacts();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<ContactableShape> rigidBodyCollidables = contactableRigidBodyCollidables.get(contactableRigidBodies.get(i));
         List<DetectedContactPoint> visualization = contactPoints.get(contactableRigidBodies.get(i));

         List<FramePoint3DReadOnly> contactPoints = new ArrayList<>();
         PlanarRegion contactingRegion = null;

         for (int j = 0; j < rigidBodyCollidables.size(); j++)
         {
            BoundingBox3DReadOnly shapeBoundingBox = rigidBodyCollidables.get(j).getShapeBoundingBox();

            for (int k = 0; k < planarRegionsList.getNumberOfPlanarRegions(); k++)
            {
               ConvexPolytope3DReadOnly contactablePolytope = contactableVolumeMap.get(planarRegionsList.getPlanarRegion(k));
               if (!shapeBoundingBox.intersectsExclusive(contactablePolytope.getBoundingBox()))
                  continue;

               rigidBodyCollidables.get(j).packContactPoints(contactPoints, contactThreshold.getDoubleValue(), contactablePolytope);
               if (!contactPoints.isEmpty())
               {
                  contactingRegion = planarRegionsList.getPlanarRegion(k);
                  break;
               }
            }
         }

         for (int j = 0; j < contactPoints.size(); j++)
         {
            visualization.get(j).getContactPointPosition().set(contactPoints.get(j));
            visualization.get(j).getContactPointNormal().set(contactingRegion.getNormal());
         }
      }
   }

   public HashMap<PlanarRegion, ConvexPolytope3DReadOnly> getContactableVolumeMap()
   {
      return contactableVolumeMap;
   }
}
