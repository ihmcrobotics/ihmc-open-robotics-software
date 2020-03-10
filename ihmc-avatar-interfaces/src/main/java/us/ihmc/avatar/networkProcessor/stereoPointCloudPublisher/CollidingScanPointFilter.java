package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.robotEnvironmentAwareness.communication.converters.ScanPointFilter;

public class CollidingScanPointFilter implements ScanPointFilter
{
   private final CollisionShapeTester collisionBoxNode;

   public CollidingScanPointFilter(CollisionShapeTester collisionBoxNode)
   {
      this.collisionBoxNode = collisionBoxNode;
   }

   public void update()
   {
      collisionBoxNode.update();
   }

   @Override
   public boolean test(int index, Point3DReadOnly point)
   {
      return !collisionBoxNode.contains(point);
   }
}
