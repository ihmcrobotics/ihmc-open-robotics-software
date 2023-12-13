package us.ihmc.perception.depthData;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.depthData.collisionShapes.CollisionShape;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;

public class CollisionShapeTester
{
   private final ArrayList<TrackingCollisionShape> trackingCollisionShapes = new ArrayList<>();

   public CollisionShapeTester(FullRobotModel fullRobotModel, CollisionBoxProvider collissionBoxProvider)
   {
      addJoint(collissionBoxProvider, fullRobotModel.getRootJoint());

      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJointBasics joint : joints)
      {
         addJoint(collissionBoxProvider, joint);
      }
   }

   public CollisionShapeTester()
   {}

   public void addJoint(CollisionBoxProvider collissionBoxProvider, JointBasics joint)
   {
      List<CollisionShape> collisionMesh = collissionBoxProvider.getCollisionMesh(joint.getName());
      if (collisionMesh != null)
      {
         TrackingCollisionShape trackingCollisionShape = new TrackingCollisionShape(joint.getFrameAfterJoint(), collisionMesh);
         trackingCollisionShapes.add(trackingCollisionShape);
      }
      else
      {
         System.err.println(joint + " does not have a collission mesh");
      }
   }

   public void update()
   {
      for (int i = 0; i < trackingCollisionShapes.size(); i++)
      {
         trackingCollisionShapes.get(i).update();
      }
   }
   
   public boolean contains(Point3DReadOnly point)
   {
      for (int i = 0; i < trackingCollisionShapes.size(); i++)
      {
         if(trackingCollisionShapes.get(i).contains(point))
         {
            return true;
         }
      }
      return false;
   }

   public boolean contains(PlanarRegion region)
   {
      ConvexPolytope3D regionShape = new ConvexPolytope3D();
      for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
      {
         Point3D vertex = new Point3D(region.getConvexHull().getVertex(i));
         region.transformFromLocalToWorld(vertex);
         regionShape.addVertex(vertex);
      }

      for (int i = 0; i < trackingCollisionShapes.size(); i++)
      {
         if (trackingCollisionShapes.get(i).intersects(regionShape))
         {
            return true;
         }
      }
      return false;
   }
}
