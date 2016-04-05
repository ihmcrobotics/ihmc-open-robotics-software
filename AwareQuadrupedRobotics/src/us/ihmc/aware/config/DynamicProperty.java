package us.ihmc.aware.config;

import java.util.ArrayList;
import java.util.List;

public abstract class DynamicProperty
{
   private final String path;
   private final List<DynamicPropertyChangedListener> changedListeners = new ArrayList<>();

   public DynamicProperty(DynamicPropertyRegistry registry, String path)
   {
      this.path = path;
      registry.register(this);
   }

   public void addChangedListener(DynamicPropertyChangedListener listener)
   {
      if (listener != null)
      {
         changedListeners.add(listener);
      }
   }

   protected void notifyChangedListeners()
   {
      for (DynamicPropertyChangedListener changedListener : changedListeners)
      {
         changedListener.onPropertyChanged(this);
      }
   }

   public abstract String dump();

   public String getPath()
   {
      return path;
   }
}
