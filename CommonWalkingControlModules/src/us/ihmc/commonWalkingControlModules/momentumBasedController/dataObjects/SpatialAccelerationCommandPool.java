package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.ArrayList;

import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class SpatialAccelerationCommandPool
{
   private final ArrayList<SpatialAccelerationCommand> spatialAccelerationCommandUnusedPool = new ArrayList<SpatialAccelerationCommand>();
   private final ArrayList<SpatialAccelerationCommand> spatialAccelerationCommandUsedPool = new ArrayList<SpatialAccelerationCommand>();

   public SpatialAccelerationCommand getUnusedDesiredSpatialAccelerationCommand()
   {
      if (spatialAccelerationCommandUnusedPool.isEmpty())
      {
         SpatialAccelerationCommand commandToReturn = new SpatialAccelerationCommand();
         spatialAccelerationCommandUsedPool.add(commandToReturn);
         return commandToReturn;
      }

      else
      {
         int lastIndex = spatialAccelerationCommandUnusedPool.size() - 1;
         SpatialAccelerationCommand commandToReturn = spatialAccelerationCommandUnusedPool.get(lastIndex);
         spatialAccelerationCommandUnusedPool.remove(lastIndex);

         spatialAccelerationCommandUsedPool.add(commandToReturn);

         return commandToReturn;
      }
   }

   public void recycleObjectPool()
   {
      while (!spatialAccelerationCommandUsedPool.isEmpty())
      {
         int lastIndex = spatialAccelerationCommandUsedPool.size() - 1;
         SpatialAccelerationCommand removedCommand = spatialAccelerationCommandUsedPool.remove(lastIndex);

         spatialAccelerationCommandUnusedPool.add(removedCommand);
      }
   }

   public SpatialAccelerationCommand getUnusedDesiredSpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      SpatialAccelerationCommand commandToReturn = getUnusedDesiredSpatialAccelerationCommand();
      commandToReturn.set(jacobian, taskspaceConstraintData);
      return commandToReturn;
   }
}
