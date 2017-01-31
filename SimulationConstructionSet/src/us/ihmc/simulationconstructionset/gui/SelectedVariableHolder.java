package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class SelectedVariableHolder
{
   private YoVariable<?> var;
   private final ArrayList<ChangeListener> listeners = new ArrayList<ChangeListener>();

   public SelectedVariableHolder()
   {
   }

   public void setSelectedVariable(YoVariable<?> var)
   {
      this.var = var;

      for (ChangeListener listener : listeners)
      {
         if (listener != null)
            listener.stateChanged(new ChangeEvent(this));
      }
   }

   public YoVariable<?> getSelectedVariable()
   {
      return this.var;
   }

   public void addChangeListener(ChangeListener listener)
   {
      listeners.add(listener);
   }
}
