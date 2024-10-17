package us.ihmc.humanoidOperatorInterface.footstep.footstepSnapper;

import us.ihmc.euclid.geometry.ConvexPolygon2D;

/**
 * Created by agrabertilton on 1/20/15.
 */
public class QuadTreeFootstepSnappingParameters
{
   protected ConvexPolygon2D collisionPolygon;
   protected ConvexPolygon2D supportPolygon;
   protected double boundingSquareSizeLength;
   protected double maxAngle;
   protected double minArea;
   protected double zDistanceTolerance;

   protected int badnumberOfPointsthreshold = 1;


   private double distanceAdjustment = 0;
   private double angleAdjustment = 0;

   public QuadTreeFootstepSnappingParameters(ConvexPolygon2D collisionPolygon, ConvexPolygon2D supportPolygon, double boundingSquareSizeLength, double maxAngle, double minArea, double zDistanceTolerance){
      this(collisionPolygon, supportPolygon, boundingSquareSizeLength, maxAngle, minArea, zDistanceTolerance, 0.0, 0.0, 1);
   }

   public QuadTreeFootstepSnappingParameters(ConvexPolygon2D collisionPolygon, ConvexPolygon2D supportPolygon, double boundingSquareSizeLength, double maxAngle, double minArea, double zDistanceTolerance, double distanceAdjustment, double angleAdjustment, int badnumberOfPointsthreshold){
      this.collisionPolygon = collisionPolygon;
      this.supportPolygon = supportPolygon;
      this.boundingSquareSizeLength = boundingSquareSizeLength;
      this.maxAngle = maxAngle;
      this.minArea = minArea;
      this.zDistanceTolerance = zDistanceTolerance;
      this.distanceAdjustment = distanceAdjustment;
      this.angleAdjustment = angleAdjustment;
      this.badnumberOfPointsthreshold = badnumberOfPointsthreshold;
   }

   public ConvexPolygon2D getCollisionPolygon()
   {
      return collisionPolygon;
   }
   public ConvexPolygon2D getSupportPolygon()
   {
      return supportPolygon;
   }
   public double getBoundingSquareSizeLength()
   {
      return boundingSquareSizeLength;
   }
   public double getMaxAngle()
   {
      return maxAngle;
   }
   public double getMinArea()
   {
      return minArea;
   }
   public double getZDistanceTolerance()
   {
      return zDistanceTolerance;
   }
   public double getDistanceAdjustment()
   {
      return distanceAdjustment;
   }
   public double getAngleAdjustment()
   {
      return angleAdjustment;
   }
   public int getBadnumberOfPointsthreshold()
   {
      return badnumberOfPointsthreshold;
   }


   public void updateParameters(QuadTreeFootstepSnappingParameters newParameters){
      collisionPolygon = newParameters.getCollisionPolygon();
      supportPolygon = newParameters.getSupportPolygon();
      boundingSquareSizeLength = newParameters.getBoundingSquareSizeLength();
      maxAngle = newParameters.getMaxAngle();
      minArea = newParameters.getMinArea();
      zDistanceTolerance = newParameters.getZDistanceTolerance();
      this.distanceAdjustment = newParameters.getDistanceAdjustment();
      this.angleAdjustment = newParameters.getAngleAdjustment();
      this.badnumberOfPointsthreshold = newParameters.getBadnumberOfPointsthreshold();
   }
}
