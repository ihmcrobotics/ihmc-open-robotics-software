package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.List;

public class ProcessModel
{
   private final List<ProcessModelElementGroup> processModelElementGroups;

   public ProcessModel(List<ProcessModelElementGroup> processModelElementGroups)
   {
      this.processModelElementGroups = processModelElementGroups;
   }
   
   public void update()
   {
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         processModelElementGroup.update();
      }
      
      // TODO: assemble
   }
}
