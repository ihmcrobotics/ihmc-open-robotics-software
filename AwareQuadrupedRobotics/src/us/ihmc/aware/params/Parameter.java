package us.ihmc.aware.params;

import java.util.ArrayList;
import java.util.List;

public abstract class Parameter
{
   private final String path;
   private final List<ParameterChangeListener> changedListeners = new ArrayList<>();

   public Parameter(String path)
   {
      this.path = path;
      ParameterRegistry.getInstance().register(this);
   }

   public void addChangedListener(ParameterChangeListener listener)
   {
      if (listener != null)
      {
         changedListeners.add(listener);
      }
   }

   protected void notifyChangedListeners()
   {
      for (ParameterChangeListener changedListener : changedListeners)
      {
         changedListener.onPropertyChanged(this);
      }
   }

   public boolean tryLoad(String line)
   {
      String[] split = line.split("=");

      if (split.length < 2)
      {
         System.err.println("Malformed configuration line: " + line);
         return false;
      }

      String path = split[0];
      String value = split[1];

      if (this.path.equals(path))
      {
         if (tryLoadValue(value))
         {
            return true;
         }
         else
         {
            System.err.println("Malformed configuration line: " + line);
            return false;
         }
      }

      return false;
   }

   public abstract boolean tryLoadValue(String value);

   public String dump()
   {
      return path + "=" + dumpValue();
   }

   protected abstract String dumpValue();

   public String getPath()
   {
      return path;
   }
}
