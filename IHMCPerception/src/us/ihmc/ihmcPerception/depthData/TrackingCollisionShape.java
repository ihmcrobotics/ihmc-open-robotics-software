package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TrackingCollisionShape
{
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

   public boolean contains(Point3d point)
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

   private class TrackingCollisionShapeImpl
   {
      private final ReferenceFrame frame;
      private final CollisionShape shape;

      private TrackingCollisionShapeImpl(ReferenceFrame frame, CollisionShape shape)
      {
         this.frame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("CollissionFrame", frame, shape.getPose());
         this.shape = shape;
      }

      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final Point3d testPoint = new Point3d();

      private void update()
      {
         frame.getTransformToDesiredFrame(transform, ReferenceFrame.getWorldFrame());
         transform.invert();
      }

      public boolean contains(Point3d point)
      {
         transform.transform(point, testPoint);
         return shape.contains(testPoint);
      }
   }
}