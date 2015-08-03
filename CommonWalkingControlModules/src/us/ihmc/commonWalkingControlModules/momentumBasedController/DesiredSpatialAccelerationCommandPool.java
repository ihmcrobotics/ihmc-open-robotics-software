package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class DesiredSpatialAccelerationCommandPool
{

   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommandUnusedPool = new ArrayList<DesiredSpatialAccelerationCommand>();
   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommandUsedPool = new ArrayList<DesiredSpatialAccelerationCommand>();
   
   
   public DesiredSpatialAccelerationCommand getUnusedDesiredSpatialAccelerationCommand()
   {
      if (desiredSpatialAccelerationCommandUnusedPool.isEmpty())
      {
         DesiredSpatialAccelerationCommand commandToReturn = new DesiredSpatialAccelerationCommand();
         desiredSpatialAccelerationCommandUsedPool.add(commandToReturn);
         return commandToReturn;
      }

      else
      {
         int lastIndex = desiredSpatialAccelerationCommandUnusedPool.size() - 1;
         DesiredSpatialAccelerationCommand commandToReturn = desiredSpatialAccelerationCommandUnusedPool.get(lastIndex);
         desiredSpatialAccelerationCommandUnusedPool.remove(lastIndex);

         desiredSpatialAccelerationCommandUsedPool.add(commandToReturn);

         return commandToReturn;
      }
   }
   
   public void recycleObjectPool()
   {
      while(!desiredSpatialAccelerationCommandUsedPool.isEmpty())
      {
         int lastIndex = desiredSpatialAccelerationCommandUsedPool.size() - 1;
         DesiredSpatialAccelerationCommand removedCommand = desiredSpatialAccelerationCommandUsedPool.remove(lastIndex);
         
         desiredSpatialAccelerationCommandUnusedPool.add(removedCommand);
      }   
   }

   public DesiredSpatialAccelerationCommand getUnusedDesiredSpatialAccelerationCommand(GeometricJacobian jacobian,
         TaskspaceConstraintData taskspaceConstraintData)
   {
      DesiredSpatialAccelerationCommand commandToReturn = getUnusedDesiredSpatialAccelerationCommand();
      commandToReturn.set(jacobian, taskspaceConstraintData);
      return commandToReturn;
   }
}
