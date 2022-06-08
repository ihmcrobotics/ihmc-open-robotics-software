package us.ihmc.commonWalkingControlModules.contact.geometry;

import us.ihmc.commonWalkingControlModules.contact.geometry.RigidBodyCollisionDescription;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.physics.Collidable;

import java.util.List;

public class ShapeCollisionDescription implements RigidBodyCollisionDescription
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3DReadOnly zDownWorld = new FrameVector3D(worldFrame, 0.0, 0.0, -1.0);

   private final Collidable collidable;
   private final FramePoint3D contactPoint = new FramePoint3D();
   private final FrameVector3D supportDirection = new FrameVector3D();
   private double heightInWorld = Double.POSITIVE_INFINITY;

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
   private final RigidBodyTransform shapeToWorldFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform worldToShapeFrameTransform = new RigidBodyTransform();

   public ShapeCollisionDescription(Collidable collidable)
   {
      this.collidable = collidable;
   }

   @Override
   public Collidable getCollidable()
   {
      return collidable;
   }

   @Override
   public double updateHeightInWorld()
   {
      supportDirection.setIncludingFrame(zDownWorld);
      supportDirection.changeFrame(collidable.getShape().getReferenceFrame());
      collidable.getShape().getSupportingVertex(supportDirection, contactPoint);
      contactPoint.changeFrame(worldFrame);
      heightInWorld = contactPoint.getZ();
      return heightInWorld;
   }

   @Override
   public double getHeightInWorld()
   {
      return heightInWorld;
   }

   @Override
   public double packContactPoints(List<FramePoint3DReadOnly> contactPoints, double distanceThreshold, PlanarRegion planarRegion)
   {
      // Collision check in shape frame
      FrameShape3DReadOnly collisionShape = collidable.getShape();
      ReferenceFrame collisionShapeFrame = collisionShape.getReferenceFrame();
      collisionShapeFrame.getTransformToDesiredFrame(shapeToWorldFrameTransform, ReferenceFrame.getWorldFrame());
      worldToShapeFrameTransform.setAndInvert(shapeToWorldFrameTransform);

      planarRegion.applyTransform(worldToShapeFrameTransform);
      planarRegion.update();

      collisionDetector.evaluateCollision(planarRegion, collisionShape, collisionResult);

      planarRegion.applyTransform(shapeToWorldFrameTransform);
      planarRegion.update();

      if (collisionResult.getSignedDistance() < distanceThreshold)
      {
         FramePoint3D contactPoint = new FramePoint3D(collisionShapeFrame, collisionResult.getPointOnB());
         contactPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPoints.add(contactPoint);
      }

      return collisionResult.getSignedDistance();
   }

   @Override
   public void packContactPoints(List<FramePoint3DReadOnly> contactPoints, double heightThreshold)
   {
      if (heightInWorld < heightThreshold)
      {
         contactPoints.add(contactPoint);
      }
   }
}
