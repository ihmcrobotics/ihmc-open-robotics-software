package us.ihmc.rdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class RayToControlRingIntersectionCalculator
{
   private final double QUARTER_TURN = Math.PI / 2.0;

   // Intersections
   private final HollowCylinderRayIntersection hollowCylinderIntersection = new HollowCylinderRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveXArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveYArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final BoxRayIntersection negativeXArrowIntersection = new BoxRayIntersection();
   private final BoxRayIntersection negativeYArrowIntersection = new BoxRayIntersection();

   // Transforms of arrows
   private final RigidBodyTransform xArrowTailTransform;
   private final RigidBodyTransform yArrowTailTransform;
   private final RigidBodyTransform temporaryTailTransform = new RigidBodyTransform();

   private final Point3D closestCollision = new Point3D();
   private double closestCollisionDistance;

   private float discOuterRadius = 0.426f;
   private float discInnerRadius = 0.290f;
   private float discThickness = 0.03f;
   private float arrowWidth = 0.257f;
   private float arrowHeight = 0.137f;
   private float arrowSpacing = 0.079f;
   private float arrowTailWidthRatio = 0.5f;
   private float arrowTailLengthRatio = 1.0f;

   public RayToControlRingIntersectionCalculator(RigidBodyTransform xArrowTailTransform,
                                                 RigidBodyTransform yArrowTailTransform,
                                                 float discOuterRadius,
                                                 float discInnerRadius,
                                                 float discThickness,
                                                 float arrowWidth,
                                                 float arrowHeight,
                                                 float arrowSpacing,
                                                 float arrowTailWidthRatio,
                                                 float arrowTailLengthRatio)
   {
      this.xArrowTailTransform = xArrowTailTransform;
      this.yArrowTailTransform = yArrowTailTransform;
      updateControlRingDimensions(discOuterRadius,
                                  discInnerRadius,
                                  discThickness,
                                  arrowWidth,
                                  arrowHeight,
                                  arrowSpacing,
                                  arrowTailWidthRatio,
                                  arrowTailLengthRatio);
   }

   public static enum CollisionType
   {
      NONE, CYLINDER, POSITIVE_X, NEGATIVE_X, POSITIVE_Y, NEGATIVE_Y
   }
   CollisionType collisionType = CollisionType.NONE;

   private void updateControlRingDimensions(float discOuterRadius,
                                            float discInnerRadius,
                                            float discThickness,
                                            float arrowWidth,
                                            float arrowHeight,
                                            float arrowSpacing,
                                            float arrowTailWidthRatio,
                                            float arrowTailLengthRatio)
   {
      this.discOuterRadius = discOuterRadius;
      this.discInnerRadius = discInnerRadius;
      this.discThickness = discThickness;
      this.arrowWidth = arrowWidth;
      this.arrowHeight = arrowHeight;
      this.arrowSpacing = arrowSpacing;
      this.arrowTailWidthRatio = arrowTailWidthRatio;
      this.arrowTailLengthRatio = arrowTailLengthRatio;
   }

   public CollisionType determineCollisionTypeFromRay(Line3DReadOnly pickRay, RigidBodyTransform controlRingTransformToWorld, boolean showArrows)
   {
      collisionType = CollisionType.NONE;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      hollowCylinderIntersection.update(discThickness, discOuterRadius, discInnerRadius, discThickness / 2.0, controlRingTransformToWorld);
      double distance = hollowCylinderIntersection.intersect(pickRay);
      if (!Double.isNaN(distance) && distance < closestCollisionDistance)
      {
         closestCollisionDistance = distance;
         collisionType = CollisionType.CYLINDER;
         closestCollision.set(hollowCylinderIntersection.getClosestIntersection());
      }
      if (showArrows)
      {
         positiveXArrowIntersection.update(arrowWidth,
                                           arrowHeight,
                                           discThickness,
                                           new Point3D(discOuterRadius + arrowSpacing, 0.0, discThickness / 2.0),
                                           new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN),
                                           controlRingTransformToWorld);
         distance = positiveXArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            collisionType = CollisionType.POSITIVE_X;
            closestCollision.set(positiveXArrowIntersection.getClosestIntersection());
         }
         positiveYArrowIntersection.update(arrowWidth,
                                           arrowHeight,
                                           discThickness,
                                           new Point3D(0.0, discOuterRadius + arrowSpacing, discThickness / 2.0),
                                           new YawPitchRoll(0.0, 0.0, -QUARTER_TURN),
                                           controlRingTransformToWorld);
         distance = positiveYArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            collisionType = CollisionType.POSITIVE_Y;
            ;
            closestCollision.set(positiveYArrowIntersection.getClosestIntersection());
         }
         temporaryTailTransform.set(xArrowTailTransform);
         controlRingTransformToWorld.transform(temporaryTailTransform);
         boolean intersects = negativeXArrowIntersection.intersect(arrowTailWidthRatio * arrowWidth,
                                                                   arrowTailLengthRatio * arrowHeight,
                                                                   discThickness,
                                                                   temporaryTailTransform,
                                                                   pickRay);
         distance = negativeXArrowIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (intersects && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            collisionType = CollisionType.NEGATIVE_X;
            closestCollision.set(negativeXArrowIntersection.getFirstIntersectionToPack());
         }
         temporaryTailTransform.set(yArrowTailTransform);
         controlRingTransformToWorld.transform(temporaryTailTransform);
         intersects = negativeYArrowIntersection.intersect(arrowTailWidthRatio * arrowWidth,
                                                           arrowTailLengthRatio * arrowHeight,
                                                           discThickness,
                                                           temporaryTailTransform,
                                                           pickRay);
         distance = negativeYArrowIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (intersects && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            collisionType = CollisionType.NEGATIVE_Y;
            closestCollision.set(negativeYArrowIntersection.getFirstIntersectionToPack());
         }
      }
      return collisionType;
   }

   public CollisionType getCollisionType()
   {
      return collisionType;
   }

   public double getClosestCollisionDistance()
   {
      return closestCollisionDistance;
   }
}
