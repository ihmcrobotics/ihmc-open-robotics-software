package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;

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

   private class TrackingCollisionShapeImpl
   {
      private final ReferenceFrame frame;
      private final CollisionShape shape;

      private TrackingCollisionShapeImpl(ReferenceFrame frame, CollisionShape shape)
      {
         this.frame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("CollissionFrame", frame, shape.getPose());
         this.shape = shape;
      }

      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final Point3D testPoint = new Point3D();

      private void update()
      {
         frame.getTransformToDesiredFrame(transform, ReferenceFrame.getWorldFrame());
         transform.invert();
      }

      public boolean contains(Point3DReadOnly point)
      {
         transform.transform(point, testPoint);
         return shape.contains(testPoint);
      }
   }
}