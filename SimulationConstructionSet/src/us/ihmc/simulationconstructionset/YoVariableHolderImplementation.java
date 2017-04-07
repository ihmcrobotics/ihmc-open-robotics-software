package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * <p>Description: An implementation of a YoVariableHolder. </p>
 */
public class YoVariableHolderImplementation implements YoVariableHolder
{
   private final LinkedHashMap<String, ArrayList<YoVariable<?>>> yoVariableSet = new LinkedHashMap<String, ArrayList<YoVariable<?>>>();

   public YoVariableHolderImplementation()
   {
   }

   @Override
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      Collection<ArrayList<YoVariable<?>>> variableLists = yoVariableSet.values();

      for (ArrayList<YoVariable<?>> list : variableLists)
      {
         for (YoVariable<?> variable : list)
         {
            ret.add(variable);
         }
      }

      return ret;
   }

   @Override
   public YoVariable<?>[] getAllVariablesArray()
   {
      ArrayList<YoVariable<?>> variables = getAllVariables();
      YoVariable<?>[] ret = new YoVariable[variables.size()];
      variables.toArray(ret);

      return ret;
   }


   /**
    *   Adds the given YoVariables to this YoVariableHolder. If any Variable is not unique, throws a RuntimeException.
    *   @param variables YoVariables to add to this YoVariableHolder
    */
   public void addVariablesToHolder(ArrayList<YoVariable<?>> variables)
   {
      for (YoVariable<?> variable : variables)
      {
         addVariableToHolder(variable);
      }
   }


   /**
    * Adds the given YoVariable to this YoVariableHolder. If this Variable is not unique, throws a RuntimeException.
    * @param variable YoVariable to add to this YoVariableHolder
    */
   public void addVariableToHolder(YoVariable<?> variable)
   {
      String lowerCaseName = variable.getName();
      lowerCaseName = lowerCaseName.toLowerCase();
      
      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(lowerCaseName);
      if (variablesWithThisName == null)
      {
         variablesWithThisName = new ArrayList<YoVariable<?>>();
         yoVariableSet.put(lowerCaseName, variablesWithThisName);
      }

      // Make sure the variable is unique:
      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         if (variablesWithThisName.get(i).hasSameFullName(variable))
         {
            System.err.println("Not a unique variable! " + variable.getFullNameWithNameSpace()
                               + " has already been added to this YoVariableHolder!. FullNames are \n");

            for (YoVariable<?> variableToPrint : variablesWithThisName)
            {
               System.err.println(variableToPrint.getFullNameWithNameSpace());
            }

          throw new RuntimeException("Not a unique variable! " + variable.getFullNameWithNameSpace() + " has already been added to this YoVariableHolder!. ");
         }
      }

      variablesWithThisName.add(variable);
   }

   public YoVariable<?> getVariableUsingFullNamespace(String fullname)
   {
      for (YoVariable<?> yoVariable : getAllVariables())
      {
         if (yoVariable.getFullNameWithNameSpace().equals(fullname))
            return yoVariable;
      }

      // not found
      String error = "Warning: " + fullname + " not found. (YoVariableHolderImplementation.getVariable)";
      System.err.println(error);

      return null;
   }

   @Override
   public YoVariable<?> getVariable(String fullname)
   {
      String name = NameSpace.stripOffNameSpaceToGetVariableName(fullname);      
      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());

      if (variablesWithThisName == null)
      {
//         String error = "Warning: " + fullname + " not found. (YoVariableHolderImplementation.getVariable)";
//         System.err.println(error);

         return null;
      }

      YoVariable<?> foundVariable = null;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);
         if (yoVariable.fullNameEndsWithCaseInsensitive(fullname))
         {
            if (foundVariable != null)
            {
               PrintTools.error(YoVariableHolderImplementation.this, "Called getVariable with " + fullname + ". That is insufficient name information to distinguish a unique variable! "
                                  + "Please include more of the name space! Already found " + foundVariable.getFullNameWithNameSpace()
                                  + ". Looking for variable " + yoVariable.getFullNameWithNameSpace());
               // new Throwable().printStackTrace(); // Use to find callers.
            }
            else
               foundVariable = yoVariable;
         }
      }

      return foundVariable;
   }

   @Override
   public YoVariable<?> getVariable(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling getVariable(String nameSpace, String name)");
      }
      
      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return null;
      }

      YoVariable<?> foundVariable = null;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoVariableRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            if (foundVariable != null)
            {
               throw new RuntimeException("Called getVariable with " + nameSpaceEnding + ", " + name
                                          + ". That is insufficient name information to distinguish a unique variable! Please include more of the name space!");
            }

            foundVariable = yoVariable;
         }
      }

      return foundVariable;
   }

   @Override
   public boolean hasUniqueVariable(String fullname)
   {
      String name = NameSpace.stripOffNameSpaceToGetVariableName(fullname);
      
      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return false;
      }

      boolean foundVariable = false;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);

         if (yoVariable.fullNameEndsWithCaseInsensitive(fullname))
         {
            if (foundVariable)
            {
               return false;
            }

            foundVariable = true;
         }
      }

      return foundVariable;
   }

   @Override
   public boolean hasUniqueVariable(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling hasVariable(String nameSpace, String name)");
      }

      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return false;
      }

      boolean foundVariable = false;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoVariableRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            if (foundVariable)
            {
               return false;
            }

            foundVariable = true;
         }
      }

      return foundVariable;
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling getVariables(String nameSpace, String name)");
      }

      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return new ArrayList<YoVariable<?>>(0);
      }

      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoVariableRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            ret.add(yoVariable);
         }
      }

      return ret;
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String fullname)
   {
      String name = NameSpace.stripOffNameSpaceToGetVariableName(fullname);

      ArrayList<YoVariable<?>> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return new ArrayList<YoVariable<?>>(0);
      }

      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable<?> yoVariable = variablesWithThisName.get(i);

         if (yoVariable.fullNameEndsWithCaseInsensitive(fullname))
         {
            ret.add(yoVariable);
         }
      }

      return ret;
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      Collection<ArrayList<YoVariable<?>>> variableLists = yoVariableSet.values();

      for (ArrayList<YoVariable<?>> list : variableLists)
      {
         for (YoVariable<?> variable : list)
         {
            if (variable.getYoVariableRegistry().getNameSpace().equals(nameSpace))
            {
               ret.add(variable);
            }
         }
      }

      return ret;
   }
}
