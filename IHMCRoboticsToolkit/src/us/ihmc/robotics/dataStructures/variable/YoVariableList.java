package us.ihmc.robotics.dataStructures.variable;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;


public class YoVariableList implements java.io.Serializable, java.lang.Comparable<YoVariableList>
{
   private static final long serialVersionUID = -393664925453518934L;
   private final ArrayList<ChangeListener> listeners;
   private final String name;
   private final ArrayList<YoVariable<?>> variables;
   private final LinkedHashMap<String, ArrayList<YoVariable<?>>> variablesMappedByName;

   public YoVariableList(String name)
   {
      this.name = name;
      this.variables = new ArrayList<YoVariable<?>>();
      this.variablesMappedByName = new LinkedHashMap<String, ArrayList<YoVariable<?>>>();

      this.listeners = new ArrayList<ChangeListener>();
   }

   public String getName()
   {
      return this.name;
   }

   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      for (int i = 0; i < variables.size(); i++)
      {
         retBuffer.append(variables.get(i).toString());
         retBuffer.append("\n");
      }

      return retBuffer.toString();
   }

   public boolean isEmpty()
   {
      return (this.variables.size() == 0);
   }

   public void addVariable(YoVariable<?> variable)
   {
      String variableName = variable.getName();
      ArrayList<YoVariable<?>> arrayList = variablesMappedByName.get(variableName);
      if (arrayList == null)
      {
         arrayList = new ArrayList<YoVariable<?>>(1);
         variablesMappedByName.put(variable.getName(), arrayList);
      }

      if (!arrayList.contains(variable))
      {
         variables.add(variable);
         arrayList.add(variable);

         notifyListeners();
      }
   }

   /**
    * Tell all listeners that a change in the variable count occurred.  At time of writing
    * only VarPanelsHolders are listeners for this.
    */
   private void notifyListeners()
   {
      for (ChangeListener listener : listeners)
      {
         listener.stateChanged(new ChangeEvent(this));
      }
   }

   public void addVariables(YoVariableList controlVars)
   {
      ArrayList<YoVariable<?>> variables = controlVars.getVariables();

      this.addVariables(variables);
   }

   public void addVariables(ArrayList<YoVariable<?>> list)
   {
      for (YoVariable<?> variable : list)
      {
         this.addVariable(variable);
      }
   }

   public void addVariables(YoVariable<?>[] variables)
   {
      for (int i = 0; i < variables.length; i++)
      {
         this.addVariable(variables[i]);
      }
   }

   public void removeVariable(YoVariable<?> variable)
   {
      String variableName = variable.getName();

      if (variablesMappedByName.containsKey(variableName))
      {
         ArrayList<YoVariable<?>> arrayList = variablesMappedByName.get(variableName);
         arrayList.remove(variable);
         variables.remove(variable);
      }
   }

   public void removeAllVariables()
   {
      variables.clear();
      variablesMappedByName.clear();

      notifyListeners();
   }

   public boolean containsVariable(YoVariable<?> variable)
   {
      String variableName = variable.getName();
      ArrayList<YoVariable<?>> arrayList = variablesMappedByName.get(variableName);

      if ((arrayList != null) && (arrayList.contains(variable)))
         return true;

      return false;
   }

   public ArrayList<YoVariable<?>> getVariables()
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();
      ret.addAll(this.variables);

      return ret;
   }

   public int size()
   {
      return this.variables.size();
   }

   public synchronized YoVariable<?> getVariable(int index)
   {
      if ((index <= (variables.size() - 1)) && (index >= 0))
      {
         return variables.get(index);
      }

      return null;
   }

   public synchronized YoVariable<?> getVariable(String name)
   {
      ArrayList<YoVariable<?>> arrayList;
      int lastDotIndex = name.lastIndexOf(".");

      if (lastDotIndex == -1)
      {
         arrayList = variablesMappedByName.get(name);
      }
      else
      {
         String endOfName = name.substring(lastDotIndex + 1);
         arrayList = variablesMappedByName.get(endOfName);
      }

      if (arrayList == null)
         return null;

      for (int i = 0; i < arrayList.size(); i++)
      {
         YoVariable<?> variable = arrayList.get(i);

         if (variable.fullNameEndsWithCaseInsensitive(name))
         {
            return variable;
         }
      }

      return null;
   }

   public synchronized String[] getVariableNames()
   {
      String[] ret = new String[variables.size()];

      for (int i = 0; i < variables.size(); i++)
      {
         ret[i] = variables.get(i).getName();
      }

      return ret;
   }

   public synchronized boolean hasVariableWithName(String name)
   {
      if (getVariable(name) != null)
      {
         return true;
      }

      return false;
   }

   // TODO: duplicated in YoVariableRegistry
   public synchronized ArrayList<YoVariable<?>> getMatchingVariables(String[] names, String[] regularExpressions)
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      if (names != null)
      {
         for (int i = 0; i < names.length; i++)
         {
            String name = names[i];
            YoVariable<?> var = getVariable(name);

            if (var != null)
            {
               ret.add(var);
            }
         }
      }

      if (regularExpressions != null)
      {
         for (int i = 0; i < regularExpressions.length; i++)
         {
            Pattern pattern = Pattern.compile(regularExpressions[i]);

            for (int j = 0; j < variables.size(); j++)
            {
               YoVariable<?> var = variables.get(j);
               Matcher matcher = pattern.matcher(var.getName());

               if (matcher.matches())
               {
                  ret.add(var);
               }
            }
         }
      }

      return ret;
   }

   public YoVariable<?>[] getAllVariables()
   {
      YoVariable<?>[] ret = new YoVariable[variables.size()];

      variables.toArray(ret);

      return ret;
   }

   public void addChangeListener(ChangeListener listener)
   {
      listeners.add(listener);
   }

   /**
    * //  * Compares this VarList to the specified object returning > = < as 1 0 -1 respectively.
    * //  * Reference object must be another VarList otherwise a runtime exception will be thrown.
    * /
    * //  * @param other Object to which this will be compared
    * //  * @return indicates > = < as 1 0 -1 respectively
    * /
    */
   public int compareTo(YoVariableList other)
   {
      return (int) Math.signum(this.getName().compareToIgnoreCase(other.getName()));

   }

   public synchronized int getIndexOfVariable(YoVariable<?> variable)
   {
      return variables.indexOf(variable);
   }
}
