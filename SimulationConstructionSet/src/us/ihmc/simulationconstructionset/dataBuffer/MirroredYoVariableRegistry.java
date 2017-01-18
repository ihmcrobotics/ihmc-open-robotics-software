package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class MirroredYoVariableRegistry extends YoVariableRegistry
{
   private final BiMap<YoVariable<?>, YoVariable<?>> variableMap = HashBiMap.create();
   private final ConcurrentLinkedQueue<YoVariable<?>> changedVariablesInMirror = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<YoVariable<?>> changedVariablesInOriginal = new ConcurrentLinkedQueue<>();

   private final YoVariableRegistryChangedListener mirroredChangeListener = new YoVariableRegistryChangedListener(changedVariablesInMirror);
   private final YoVariableRegistryChangedListener originalChangeListener = new YoVariableRegistryChangedListener(changedVariablesInOriginal);

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
         variableMap.put(var, newVar);
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
      newVar.addVariableChangedListener(mirroredChangeListener);
      var.addVariableChangedListener(originalChangeListener);
   }

   /**
    * Updates changes from the mirror to the original registry and then from the original to the mirror registry
    */
   public void updateMirror()
   {
      updateChangedValues();
      updateValuesFromOriginal();
   }

   /**
    * Mirrors changes from the mirror registry to the original registry
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   public void updateChangedValues()
   {
      for (YoVariable changed = changedVariablesInMirror.poll(); changed != null; changed = changedVariablesInMirror.poll())
      {
         YoVariable originalVar = variableMap.inverse().get(changed);
         originalVar.setValue(changed, false);
         callListenersForVariable(originalVar);
      }
   }

   /**
    * Mirrors changes from the original registry to the mirror registry
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   public void updateValuesFromOriginal()
   {
      for (YoVariable changed = changedVariablesInOriginal.poll(); changed != null; changed = changedVariablesInOriginal.poll())
      {
         YoVariable mirroredVar = variableMap.get(changed);
         mirroredVar.setValue(changed, false);
         callListenersForVariable(mirroredVar);
      }
   }

   private void callListenersForVariable(YoVariable<?> variable) {
      ArrayList<VariableChangedListener> variableChangedListeners = variable.getVariableChangedListeners();
      //noinspection ForLoopReplaceableByForEach (runs in tight loop, foreach allocates memory)
      for (int i = 0; i < variableChangedListeners.size(); i++)
      {
         VariableChangedListener variableChangedListener = variableChangedListeners.get(i);
         if (variableChangedListener.getClass() != YoVariableRegistryChangedListener.class)
         {
            variableChangedListener.variableChanged(variable);
         }
      }
   }

   private static class YoVariableRegistryChangedListener implements VariableChangedListener
   {
      final ConcurrentLinkedQueue<YoVariable<?>> queue;

      private YoVariableRegistryChangedListener(ConcurrentLinkedQueue<YoVariable<?>> queue)
      {
         this.queue = queue;
      }

      @Override
      public void variableChanged(YoVariable<?> v)
      {
         queue.add(v);
      }
   }
}
