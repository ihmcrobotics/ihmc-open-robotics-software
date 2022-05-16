package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class BoundingBoxCollisionDetector
{
   private List<PlanarRegion> planarRegionsList;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

   private double boxDepth = Double.NaN;
   private double boxWidth = Double.NaN;
   private double boxHeight = Double.NaN;

   private double bodyPoseX = Double.NaN;
   private double bodyPoseY = Double.NaN;
   private double bodyPoseZ = Double.NaN;
   private double bodyPoseYaw = Double.NaN;

   // box that is checked for collision
   private final Box3D bodyBox = new Box3D();
   // this is the bounding box of "bodyBox", used as an optimization before doing a full collision check
   private final BoundingBox3D boundingBox = new BoundingBox3D();
   // Collision result for each query between the bounding box and planar region
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

   public void setPlanarRegionsList(PlanarRegionsList planarRegions)
   {
      setPlanarRegionsList(planarRegions.getPlanarRegionsAsList());
   }

   public void setPlanarRegionsList(List<PlanarRegion> planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   public void setBoxDimensions(double boxDepth, double boxWidth, double boxHeight)
   {
      this.boxDepth = boxDepth;
      this.boxWidth = boxWidth;
      this.boxHeight = boxHeight;
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
      BodyCollisionData collisionData = new BodyCollisionData();
      checkForCollision(collisionData);

      return collisionData;
   }

   public boolean checkForCollision(BodyCollisionData collisionDataToPack)
   {
      checkInputs();
      setBoundingBoxPosition();
      setDimensions();

      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.get(i);

         if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(bodyBox.getBoundingBox()))
         {
            collisionDetector.evaluateCollision(planarRegion, bodyBox, collisionResult);
            if (collisionResult.areShapesColliding())
            {
               collisionDataToPack.setCollisionDetected(true);
               collisionDataToPack.setCollisionResult(collisionResult);
               collisionDataToPack.getPlanarRegion().set(planarRegion);
               collisionDataToPack.getBodyBox().set(bodyBox);
               break;
            }
         }
      }

      return collisionDataToPack.isCollisionDetected();
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
      bodyBox.getPose().getTranslation().set(bodyPoseX, bodyPoseY, bodyPoseZ + 0.5 * boxHeight);
      bodyBox.getPose().getRotation().setYawPitchRoll(bodyPoseYaw, 0.0, 0.0);
   }

   private void setDimensions()
   {
      bodyBox.getSize().set(boxDepth, boxWidth, boxHeight);
      bodyBox.getBoundingBox(boundingBox);
   }

   private void checkInputs()
   {
      if (Double.isNaN(boxDepth) || Double.isNaN(boxWidth) || Double.isNaN(boxHeight))
         throw new RuntimeException("Bounding box dimensions has not been set");

      if(Double.isNaN(bodyPoseX) || Double.isNaN(bodyPoseY) || Double.isNaN(bodyPoseZ) || Double.isNaN(bodyPoseYaw))
         throw new RuntimeException("Bounding box position has not been set");
   }
}
