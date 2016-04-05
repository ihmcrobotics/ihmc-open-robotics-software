package us.ihmc.aware.config;

import java.util.ArrayList;
import java.util.List;

public abstract class Property
{
   private final String path;
   private final List<PropertyChangedListener> changedListeners = new ArrayList<>();

   public Property(String path)
   {
      this.path = path;
   }

   public void addChangedListener(PropertyChangedListener listener)
   {
      if (listener != null)
      {
         changedListeners.add(listener);
      }
   }

   protected void notifyChangedListeners()
   {
      for (PropertyChangedListener changedListener : changedListeners)
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
