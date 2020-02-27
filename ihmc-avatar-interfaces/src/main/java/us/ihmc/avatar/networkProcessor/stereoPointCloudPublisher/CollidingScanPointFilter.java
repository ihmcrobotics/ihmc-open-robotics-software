package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;

public class CollidingScanPointFilter implements ScanPointFilter
{
   private final CollisionShapeTester collisionBoxNode;
   private final TIntArrayList collidingPointIndices = new TIntArrayList();

   private int collidingPointBufferIndex = 0;
   private int nextCollidingScanPointIndex = -1;

   public CollidingScanPointFilter(CollisionShapeTester collisionBoxNode)
   {
      this.collisionBoxNode = collisionBoxNode;
   }

   public void updateFilter(ColorPointCloudData colorPointCloudData)
   {
      collidingPointIndices.reset();
      collidingPointBufferIndex = 0;
      nextCollidingScanPointIndex = -1;

      if (collisionBoxNode == null || colorPointCloudData == null)
         return;

      collisionBoxNode.update();

      Point3D[] pointCloud = colorPointCloudData.getPointCloud();

      for (int i = 0; i < colorPointCloudData.getNumberOfPoints(); i++)
      {
         if (collisionBoxNode.contains(pointCloud[i]))
            collidingPointIndices.add(i);
      }

      nextCollidingScanPointIndex = collidingPointIndices.get(0);
   }

   @Override
   public boolean test(int index, Point3DReadOnly point)
   {
      if (index == -1)
         return true;

      if (index == nextCollidingScanPointIndex)
      {
         collidingPointBufferIndex++;
         if (collidingPointBufferIndex < collidingPointIndices.size())
            nextCollidingScanPointIndex = collidingPointIndices.get(collidingPointBufferIndex);
         return false;
      }

      return true;
   }
}
