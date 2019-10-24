package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class BoundingBoxCollisionDetector
{
   private final double boxDepth;
   private final double boxWidth;
   private final double boxHeight;
   private final double xyProximityCheck;
   
   private PlanarRegionsList planarRegionsList;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private double bodyPoseX = Double.NaN;
   private double bodyPoseY = Double.NaN;
   private double bodyPoseZ = Double.NaN;
   private double bodyPoseYaw = Double.NaN;

   // box that is checked for collision
   private final Box3D bodyBox = new Box3D();
   // this is the bounding box of "bodyBox", used as an optimization before doing a full collision check
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Point3D tempPoint1 = new Point3D();

   public BoundingBoxCollisionDetector(double boxDepth, double boxWidth, double boxHeight, double xyProximityCheck)
   {
      this.boxDepth = boxDepth;
      this.boxWidth = boxWidth;
      this.boxHeight = boxHeight;
      this.xyProximityCheck = xyProximityCheck;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
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

         if(planarRegion.getBoundingBox3dInWorld().intersectsExclusive(bodyBox.getBoundingBox()))
         {
            PlanarRegion region = planarRegionsList.getPlanarRegion(i);
            EuclidShape3DCollisionResult collisionResult = collisionDetector.evaluateCollision(region, bodyBox);

            if(collisionResult.areShapesColliding())
            {
               setDimensionsToLowerBound();
               collisionResult = collisionDetector.evaluateCollision(region, bodyBox);

               if(collisionResult.areShapesColliding())
               {
                  collisionData.setCollisionDetected(true);
                  break;
               }
               else
               {
                  tempPoint1.set(collisionResult.getPointOnA());
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
      bodyBox.getPose().setTranslation(bodyPoseX, bodyPoseY, bodyPoseZ + 0.5 * boxHeight);
      bodyBox.getPose().setRotationYawPitchRoll(bodyPoseYaw, 0.0, 0.0);
   }

   private void setDimensionsToLowerBound()
   {
      bodyBox.setSize(boxDepth, boxWidth, boxHeight);
      bodyBox.getBoundingBox(boundingBox);
   }

   private void setDimensionsToUpperBound()
   {
      double planarDimensionIncrease = 2.0 * xyProximityCheck;
      bodyBox.setSize(boxDepth + planarDimensionIncrease, boxWidth + planarDimensionIncrease, boxHeight);
      bodyBox.getBoundingBox(boundingBox);
   }

   private void checkInputs()
   {
      if(Double.isNaN(bodyPoseX) || Double.isNaN(bodyPoseY) || Double.isNaN(bodyPoseZ) || Double.isNaN(bodyPoseYaw))
         throw new RuntimeException("Bounding box position has not been set");
   }
}
