package us.ihmc.perception.depthData;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.depthData.collisionShapes.CollisionShape;

import java.util.ArrayList;
import java.util.List;

public class TrackingCollisionShape
{
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private final List<TrackingCollisionShapeImpl> geometries = new ArrayList<>();

   public TrackingCollisionShape(ReferenceFrame frame, List<CollisionShape> collisionMesh)
   {
      for (CollisionShape shape : collisionMesh)
      {
         geometries.add(new TrackingCollisionShapeImpl(frame, shape));
      }
   }

   public void update()
   {
      for (int i = 0; i < geometries.size(); i++)
      {
         geometries.get(i).update();
      }
   }

   public boolean contains(Point3DReadOnly point)
   {
      for (int i = 0; i < geometries.size(); i++)
      {

         if (geometries.get(i).contains(point))
         {
            return true;
         }
      }

      return false;
   }

   public boolean intersects(ConvexPolytope3DReadOnly shape)
   {
      for (int i = 0; i < geometries.size(); i++)
      {
         if (geometries.get(i).intersects(shape))
         {
            return true;
         }
      }

      return false;
   }

   private class TrackingCollisionShapeImpl
   {
      private final ReferenceFrame frame;
      private final CollisionShape shape;
      private final Shape3DReadOnly shape3D;
      private final BoundingBox3DReadOnly boundingBox;

      private TrackingCollisionShapeImpl(ReferenceFrame frame, CollisionShape shape)
      {
         this.frame = frame;
         this.shape = shape;
         this.shape3D = shape.getOrCreateShape3D();
         boundingBox = shape3D.getBoundingBox();
      }

      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final Point3D testPoint = new Point3D();

      private void update()
      {
         transform.set(frame.getTransformToRoot());
         transform.multiply(shape.getPose());
         transform.invert();
      }

      public boolean contains(Point3DReadOnly point)
      {
         transform.transform(point, testPoint);
         return shape.contains(testPoint);
      }

      public boolean intersects(ConvexPolytope3DReadOnly otherShape)
      {
         ConvexPolytope3D shapeInLocal = new ConvexPolytope3D(otherShape);
         shapeInLocal.applyTransform(transform);

         if (!boundingBox.intersectsExclusive(otherShape.getBoundingBox()))
            return false;

         EuclidShape3DCollisionResult result = collisionDetector.evaluateCollision(shapeInLocal, shape3D);
         if (!result.areShapesColliding())
            return false;

         return result.getSignedDistance() < -0.02;
      }
   }
}