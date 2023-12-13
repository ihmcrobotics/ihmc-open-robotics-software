package us.ihmc.perception.filters;

import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.depthData.CollisionShapeTester;

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
