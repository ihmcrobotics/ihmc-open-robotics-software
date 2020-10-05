package us.ihmc.simulationConstructionSetTools.util.perturbance;

import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.simulationconstructionset.scripts.DisturbanceScriptEntry;
import us.ihmc.simulationconstructionset.scripts.Script;

public class DisturbanceScript implements Script
{
   private final YoRegistry registry;
   private final ForcePerturbable forcePerturbable;
   private ArrayList<DisturbanceScriptEntry> sortedDisturbanceScriptEntryList = new ArrayList<DisturbanceScriptEntry>();
   private final YoInteger nextDisturbanceScriptIndex;
   
   public DisturbanceScript(String name, ForcePerturbable forcePerturbable, YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(name);
      this.forcePerturbable = forcePerturbable;
      this.nextDisturbanceScriptIndex = new YoInteger("nextDisturbanceScriptIndex", registry);
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
