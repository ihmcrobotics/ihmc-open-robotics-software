package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class BoundingBoxCollisionDetector
{
   private final double boxDepth;
   private final double boxWidth;
   private final double boxHeight;
   private final double xyProximityCheck;

   private PlanarRegionsList planarRegionsList;
   private final List<ConvexPolytope> planarRegionPolytopes = new ArrayList<>();
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private double bodyPoseX = Double.NaN;
   private double bodyPoseY = Double.NaN;
   private double bodyPoseZ = Double.NaN;
   private double bodyPoseYaw = Double.NaN;

   private final Box3D bodyBox = new Box3D();
   private final BoundingBox3D boundingBox = new BoundingBox3D();
   private final ConvexPolytope bodyBoxPolytope = new ConvexPolytope();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Point3D tempPoint1 = new Point3D();
   private final Point3D tempPoint2 = new Point3D();

   public BoundingBoxCollisionDetector(double boxDepth, double boxWidth, double boxHeight, double xyProximityCheck)
   {
      this.boxDepth = boxDepth;
      this.boxWidth = boxWidth;
      this.boxHeight = boxHeight;
      this.xyProximityCheck = xyProximityCheck;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegions)
   {
      planarRegionPolytopes.clear();
      this.planarRegionsList = planarRegions;

      List<PlanarRegion> planarRegionsList = planarRegions.getPlanarRegionsAsList();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.get(i);
         ConvexPolytope planarRegionPolytope = new ConvexPolytope();

         List<? extends Point2DReadOnly> pointsInPlanarRegion = planarRegion.getConvexHull().getVertexBufferView();
         planarRegion.getTransformToWorld(tempTransform);

         for (Point2DReadOnly point : pointsInPlanarRegion)
         {
            tempPoint1.set(point.getX(), point.getY(), 0.0);
            tempPoint1.applyTransform(tempTransform);
            planarRegionPolytope.addVertex(tempPoint1.getX(), tempPoint1.getY(), tempPoint1.getZ());
         }

         planarRegionPolytopes.add(planarRegionPolytope);
      }
   }

   public void setBoxPose(double bodyPoseX, double bodyPoseY, double bodyPoseZ, double bodyPoseYaw)
   {
      this.bodyPoseX = bodyPoseX;
      this.bodyPoseY = bodyPoseY;
      this.bodyPoseZ = bodyPoseZ;
      this.bodyPoseYaw = bodyPoseYaw;
   }

   public BodyCollisionData checkForCollision()
   {
      checkInputs();

      setBoundingBoxPosition();
      BodyCollisionData collisionData = new BodyCollisionData();

      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         setDimensionsToUpperBound();
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);

         if(planarRegion.getBoundingBox3dInWorld().intersectsExclusive(boundingBox))
         {
            ConvexPolytope planarRegionPolytope = planarRegionPolytopes.get(i);
            if(collisionDetector.arePolytopesColliding(planarRegionPolytope, bodyBoxPolytope, tempPoint1, tempPoint2))
            {
               setDimensionsToLowerBound();

               if(collisionDetector.arePolytopesColliding(planarRegionPolytope, bodyBoxPolytope, tempPoint1, tempPoint2))
               {
                  collisionData.setCollisionDetected(true);
                  break;
               }
               else
               {
                  tempTransform.setTranslationAndIdentityRotation(bodyPoseX, bodyPoseY, bodyPoseZ);
                  tempTransform.setRotationYaw(bodyPoseYaw);
                  tempTransform.invert();
                  tempTransform.transform(tempPoint1);

                  double dx = Math.abs(tempPoint1.getX()) - 0.5 * boxDepth;
                  double dy = Math.abs(tempPoint1.getY()) - 0.5 * boxWidth;
                  collisionData.setDistanceFromBoundingBox(getMinimumPositiveValue(dx, dy, collisionData.getDistanceFromBoundingBox()));

                  if(tempPoint1.getX() < 0.0 && dx > 0.0)
                  {
                     collisionData.setDistanceOfClosestPointInBack(getMinimumPositiveValue(dx, collisionData.getDistanceOfClosestPointInBack()));
                  }
                  else if(tempPoint1.getX() > 0.0 && dx > 0.0)
                  {
                     collisionData.setDistanceOfClosestPointInFront(getMinimumPositiveValue(dx, collisionData.getDistanceOfClosestPointInFront()));
                  }
               }
            }
         }
      }

      return collisionData;
   }

   private static double getMinimumPositiveValue(double... values)
   {
      double min = Double.POSITIVE_INFINITY;
      for (int i = 0; i < values.length; i++)
      {
         if(Double.isNaN(values[i]) || values[i] < 0.0)
            continue;
         if(values[i] > 0.0)
            min = Math.min(min, values[i]);
      }

      if(Double.isInfinite(min))
         return Double.NaN;
      else
         return min;
   }
   
   private void setBoundingBoxPosition()
   {
      bodyBox.setPosition(bodyPoseX, bodyPoseY, bodyPoseZ + 0.5 * boxHeight);
      bodyBox.setOrientationYawPitchRoll(bodyPoseYaw, 0.0, 0.0);
   }

   private void setDimensionsToLowerBound()
   {
      bodyBox.setSize(boxDepth, boxWidth, boxHeight);
      bodyBox.getBoundingBox3D(boundingBox);
      updateBodyBoxPolytope();
   }

   private void setDimensionsToUpperBound()
   {
      double planarDimensionIncrease = 2.0 * xyProximityCheck;
      bodyBox.setSize(boxDepth + planarDimensionIncrease, boxWidth + planarDimensionIncrease, boxHeight);
      bodyBox.getBoundingBox3D(boundingBox);
      updateBodyBoxPolytope();
   }

   private void updateBodyBoxPolytope()
   {
      bodyBoxPolytope.getVertices().clear();

      Point3D[] vertices = bodyBox.getVertices();
      for (int i = 0; i < vertices.length; i++)
      {
         bodyBoxPolytope.addVertex(vertices[i]);
      }
   }

   private void checkInputs()
   {
      if(Double.isNaN(bodyPoseX) || Double.isNaN(bodyPoseY) || Double.isNaN(bodyPoseZ) || Double.isNaN(bodyPoseYaw))
         throw new RuntimeException("Bounding box position has not been set");
   }
}
