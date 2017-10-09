package us.ihmc.simulationconstructionset.scripts;

import java.util.ArrayList;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class ConditionalScriptEntry
{
   @SuppressWarnings("unused")
   private final String name;

   public abstract boolean startCondition();

   public abstract void activate();

   public abstract void doActivity();

   public abstract void deactivate();

   public abstract boolean finishCondition();

   public static int eventNumber = 0;

   public static final int
      DISARMED = 0, ARMED = 1, ACTIVE = 2, FINISHED = 3;
   protected YoDouble event_state;
   private ArrayList<ConditionalScriptEntry> parentEntries;
   private ArrayList<ConditionalScriptEntry> childEntries;

   public ConditionalScriptEntry(String name, YoVariableRegistry registry, ConditionalScriptEntry[] parents)
   {
      if (name == null)
         name = "event";
      this.name = name;

      event_state = new YoDouble(name + "_" + eventNumber + "_state", registry);
      event_state.set(ARMED);    // Default to armed until given a parent or two.
      eventNumber++;

      if (parents != null)
      {
         for (int i = 0; i < parents.length; i++)
         {
            addParentCondition(parents[i]);
         }
      }
   }

   public ConditionalScriptEntry(YoVariableRegistry registry)
   {
      this("event", registry, (ConditionalScriptEntry[]) null);
   }

   public ConditionalScriptEntry(String name, YoVariableRegistry registry)
   {
      this(name, registry, (ConditionalScriptEntry[]) null);
   }

   public ConditionalScriptEntry(String name, YoVariableRegistry registry, ConditionalScriptEntry parent)
   {
      this(name, registry, new ConditionalScriptEntry[] {parent});
   }



   public ConditionalScriptEntry(YoVariableRegistry registry, ConditionalScriptEntry[] parents)
   {
      this("event", registry, parents);
   }

   public void addParentCondition(ConditionalScriptEntry parentEntry)
   {
      if (parentEntry == null)
         return;

      event_state.set(DISARMED);    // Disarmed until parents are finished;

      if (parentEntries == null)
         parentEntries = new ArrayList<ConditionalScriptEntry>();
      parentEntries.add(parentEntry);

      if (parentEntry.childEntries == null)
         parentEntry.childEntries = new ArrayList<ConditionalScriptEntry>();
      parentEntry.childEntries.add(this);
   }

   protected void removeParentsAndChildren()
   {
      for (int i = 0; i < parentEntries.size(); i++)
      {
         ConditionalScriptEntry parentEntry = parentEntries.get(i);
         parentEntry.removeChild(this);
      }

      for (int i = 0; i < childEntries.size(); i++)
      {
         ConditionalScriptEntry childEntry = childEntries.get(i);
         childEntry.removeParent(this);
      }

   }

   protected void considerArmingChildren()
   {
      if (childEntries == null)
         return;

      for (int i = 0; i < childEntries.size(); i++)
      {
         ConditionalScriptEntry childEntry = childEntries.get(i);
         if (childEntry.areAllParentsFinished())
         {
            if (!childEntry.isDisarmed())
               System.err.println("Inconsistent state with ConditionalScriptEntry!");

            childEntry.event_state.set(ARMED);
         }
      }
   }

   public boolean areAllParentsFinished()
   {
      if (parentEntries == null)
         return true;

      for (int i = 0; i < parentEntries.size(); i++)
      {
         ConditionalScriptEntry parentEntry = parentEntries.get(i);

         if (!parentEntry.isFinished())
            return false;
      }

      return true;
   }

   protected void removeChild(ConditionalScriptEntry childEntry)
   {
      childEntries.remove(childEntry);
      childEntry.parentEntries.remove(this);
      if (childEntry.parentEntries.isEmpty() && childEntry.isDisarmed())
         childEntry.event_state.set(ConditionalScriptEntry.ARMED);
   }

   protected void removeParent(ConditionalScriptEntry parentEntry)
   {
      parentEntries.remove(parentEntry);
      parentEntry.childEntries.remove(this);
      if (parentEntries.isEmpty() && this.isDisarmed())
         event_state.set(ARMED);
   }


   public boolean isDisarmed()
   {
      return (event_state.getDoubleValue() == DISARMED);
   }

   public boolean isArmed()
   {
      return (event_state.getDoubleValue() == ARMED);
   }

   public boolean isActive()
   {
      return (event_state.getDoubleValue() == ACTIVE);
   }

   public boolean isFinished()
   {
      return (event_state.getDoubleValue() == FINISHED);
   }

   public void reset()
   {
//    if (event_state.val != FINISHED)
//    {
//       System.out.println("ConditionalScriptEntry " + name + " is not finished, yet you are resetting it!");
//    }

      if ((parentEntries == null) || parentEntries.isEmpty())
      {
         event_state.set(ARMED);
      }

      else
      {
         event_state.set(DISARMED);
      }
   }

}
