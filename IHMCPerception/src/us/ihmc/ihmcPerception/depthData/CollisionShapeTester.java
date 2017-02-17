package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class CollisionShapeTester
{
   private final ArrayList<TrackingCollisionShape> trackingCollisionShapes = new ArrayList<>();

   public CollisionShapeTester(FullRobotModel fullRobotModel, CollisionBoxProvider collissionBoxProvider)
   {
      addJoint(collissionBoxProvider, fullRobotModel.getRootJoint());

      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJoint joint : joints)
      {
         addJoint(collissionBoxProvider, joint);
      }
   }

   private void addJoint(CollisionBoxProvider collissionBoxProvider, InverseDynamicsJoint joint)
   {
      List<CollisionShape> collisionMesh = collissionBoxProvider.getCollisionMesh(joint.getName());
      if (collisionMesh != null)
      {
         trackingCollisionShapes.add(new TrackingCollisionShape(joint.getFrameAfterJoint(), collisionMesh));
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
   
   public boolean contains(Point3D point)
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

}
