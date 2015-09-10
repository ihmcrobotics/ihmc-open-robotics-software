package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

@SuppressWarnings("rawtypes")
public class MirroredYoVariableRegistry extends YoVariableRegistry
{
   private final ArrayList<ImmutablePair<YoVariable, YoVariable>> variablePairs = new ArrayList<ImmutablePair<YoVariable, YoVariable>>();
   private final HashMap<YoVariable, YoVariable> copyToMirroredVariables = new HashMap<YoVariable, YoVariable>();

   private final ConcurrentLinkedQueue<YoVariable> changedVariables = new ConcurrentLinkedQueue<YoVariable>();

   public MirroredYoVariableRegistry(YoVariableRegistry original)
   {
      super(original.getName());

      copyRegistry(original, this);
   }

   private void copyRegistry(YoVariableRegistry original, YoVariableRegistry target)
   {
      ArrayList<YoVariable<?>> vars = original.getAllVariablesInThisListOnly();

      for (YoVariable<?> var : vars)
      {
         YoVariable<?> newVar = var.duplicate(target);
         variablePairs.add(new ImmutablePair<YoVariable, YoVariable>(var, newVar));
         copyToMirroredVariables.put(newVar, var);
         addVariableListener(newVar, var);
      }

      for (YoVariableRegistry child : original.getChildren())
      {
         YoVariableRegistry newRegistry = new YoVariableRegistry(child.getName(), child.isLogged(), child.isSent());
         target.addChild(newRegistry);
         copyRegistry(child, newRegistry);
      }
   }

   private void addVariableListener(YoVariable<?> newVar, YoVariable<?> var)
   {
      newVar.addVariableChangedListener(new MirroredYoVariableRegistryChangedListener());
   }

   public void updateMirror()
   {
      updateChangedValues();
      updateValuesFromOriginal();
   }

   @SuppressWarnings("unchecked")
   public void updateChangedValues()
   {
      for (YoVariable<?> changed = changedVariables.poll(); changed != null; changed = changedVariables.poll())
      {
         copyToMirroredVariables.get(changed).setValue(changed, true);
      }
   }

   @SuppressWarnings("unchecked")
   public void updateValuesFromOriginal()
   {
      for (int i = 0; i < variablePairs.size(); i++)
      {
         ImmutablePair<YoVariable, YoVariable> pair = variablePairs.get(i);

         YoVariable target = pair.getRight();
         boolean changed = target.setValue(pair.getLeft(), false);

         if (changed)
         {
            ArrayList<VariableChangedListener> variableChangedListeners = target.getVariableChangedListeners();
            for (int v = 0; v < variableChangedListeners.size(); v++)
            {
               VariableChangedListener variableChangedListener = variableChangedListeners.get(v);
               if (variableChangedListener.getClass() != MirroredYoVariableRegistryChangedListener.class)
               {
                  variableChangedListener.variableChanged(target);
               }
            }
         }
      }
   }

   private class MirroredYoVariableRegistryChangedListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable<?> v)
      {
         changedVariables.add(v);
      }
   }
}
