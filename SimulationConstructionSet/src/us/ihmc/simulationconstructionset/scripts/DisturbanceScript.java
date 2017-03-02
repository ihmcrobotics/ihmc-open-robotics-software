package us.ihmc.simulationconstructionset.scripts;

import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.simulationconstructionset.util.perturbance.ForcePerturbable;

public class DisturbanceScript implements Script
{
   private final YoVariableRegistry registry;
   private final ForcePerturbable forcePerturbable;
   private ArrayList<DisturbanceScriptEntry> sortedDisturbanceScriptEntryList = new ArrayList<DisturbanceScriptEntry>();
   private final IntegerYoVariable nextDisturbanceScriptIndex;
   
   public DisturbanceScript(String name, ForcePerturbable forcePerturbable, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.forcePerturbable = forcePerturbable;
      this.nextDisturbanceScriptIndex = new IntegerYoVariable("nextDisturbanceScriptIndex", registry);
      parentRegistry.addChild(registry);
   }
   
   public void addDisturbance(DisturbanceScriptEntry disturbance)
   {
      if(disturbance == null)
         return;
      sortedDisturbanceScriptEntryList.add(disturbance);
      Collections.sort(sortedDisturbanceScriptEntryList);
   }
   
   
   @Override
   public void doScript(double t)
   {
      if (nextDisturbanceScriptIndex.getIntegerValue() >= sortedDisturbanceScriptEntryList.size()) return;
      
      DisturbanceScriptEntry disturbanceScriptEntry = sortedDisturbanceScriptEntryList.get(nextDisturbanceScriptIndex.getIntegerValue());
      
      if (t >= disturbanceScriptEntry.getTime())
      {
         forcePerturbable.setForcePerturbance(disturbanceScriptEntry.getForceVector(), disturbanceScriptEntry.getDuration());
         nextDisturbanceScriptIndex.increment();
      }
      
      forcePerturbable.resetPerturbanceForceIfNecessary();
      
      
   }
}
