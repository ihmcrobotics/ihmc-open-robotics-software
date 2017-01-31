package us.ihmc.simulationconstructionset.scripts;

import java.util.ArrayList;

public class ConditionalScript implements Script
{
   private ArrayList<ConditionalScriptEntry> conditionalScriptEntries = new ArrayList<ConditionalScriptEntry>();

   public ConditionalScript()
   {
   }


   @Override
   public void doScript(double t)
   {
      for (int i = 0; i < conditionalScriptEntries.size(); i++)
      {
         ConditionalScriptEntry entry = conditionalScriptEntries.get(i);

         if (entry.isActive())
         {
            entry.doActivity();

            if (entry.finishCondition())
            {
               entry.event_state.set(ConditionalScriptEntry.FINISHED);
               entry.deactivate();
               entry.considerArmingChildren();
            }
         }

         else if (entry.isArmed())
         {
            if (entry.startCondition())
            {
               entry.event_state.set(ConditionalScriptEntry.ACTIVE);
               entry.activate();
            }
         }
      }

   }

   public void addEntry(ConditionalScriptEntry cse)
   {
      if (cse == null)
         return;
      conditionalScriptEntries.add(cse);
   }

   public void removeEntry(ConditionalScriptEntry cse)
   {
      if (cse == null)
         return;


      cse.removeParentsAndChildren();
      conditionalScriptEntries.remove(cse);
   }

   public void reset()
   {
      for (ConditionalScriptEntry entry : conditionalScriptEntries)
      {
         entry.reset();
      }
   }

}
