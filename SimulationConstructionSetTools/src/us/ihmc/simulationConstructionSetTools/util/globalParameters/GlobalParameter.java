package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public abstract class GlobalParameter
{
   protected YoVariable<?> yoVariable;
   private ArrayList<GlobalParameter> children;

   private final boolean hasParents;

// private final ArrayList<GlobalParameter> parents;
   protected final int numberOfCharactersForDisplay = 15;
   private String paddedShortName;

   protected GlobalParameterChangedListener changedListener;

   protected static final YoVariableRegistry registry = new YoVariableRegistry("GlobalParameters");

   public static YoVariableRegistry getAllParametersYoVariableRegistry()
   {
      return registry;
   }

   public static void addVarListToSimulationConstructionSet(SimulationConstructionSet scs)
   {
      scs.addVarList(registry.createVarList());
   }


   public GlobalParameter(GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      if ((parents != null) && (parents.length > 0))
      {
         hasParents = true;

         // this.parents = new ArrayList<GlobalParameter>();
         for (GlobalParameter parent : parents)
         {
            parent.addChild(this);

            // this.parents.add(globalParameter);
         }
      }
      else
         hasParents = false;

      this.changedListener = listener;
   }



   public GlobalParameter(GlobalParameterChangedListener listener)
   {
      this.hasParents = false;
      this.changedListener = listener;
   }




// protected int getNumberOfCharactersForDisplay()
// {
//     return numberOfCharactersForDisplay;
// }

   public abstract String getValueInStringFormat();

// protected abstract void set (Object value, String comment);
// protected abstract void setOnlyIfChange(Object value, String comment);


   private void addChild(GlobalParameter childParameter)
   {
      if (children == null)
      {
         children = new ArrayList<GlobalParameter>();
      }

      this.children.add(childParameter);
   }

   public String getName()
   {
      return yoVariable.getName();
   }

   protected String getPaddedShortName()
   {
      return padWithSpaces(yoVariable.getShortName(), YoVariable.MAX_LENGTH_SHORT_NAME);
   }


   protected static String padWithSpaces(String string, int size)
   {
      if (string.length() >= size)
         return string;

      StringBuffer padded = new StringBuffer(string);
      while (padded.length() < size)
      {
         padded.append(" ");
      }

      return padded.toString();
   }

   public String getShortName()
   {
      if (this.paddedShortName == null)
         paddedShortName = getPaddedShortName();

      return this.paddedShortName;
   }


// protected ArrayList<GlobalParameter> getParents()
// {
//    if (parents == null) return null;
//
//    return new ArrayList<GlobalParameter>(parents);
// }


   public int getMaximumNumberOfCharactersInValue()
   {
      return numberOfCharactersForDisplay;
   }

// public void setOnlyIfChange(Object value)
// {
//    setOnlyIfChange(value, null);
// }


   protected void verifyNoParents()
   {
      if (!hasParents)
         return;

//    if (parents == null) return;
//    if (parents.isEmpty()) return;
//
//    String parentsString = "";
//    for (GlobalParameter parentParameter : parents)
//    {
//       parentsString = parentsString + parentParameter.getName() + ", ";
//    }

      throw new RuntimeException("Cannot set this variable directly. Set its value by setting its parents");    // : " + parentsString);
   }

   protected void updateChildren(String comment)
   {
      if (children == null)
         return;

      for (GlobalParameter globalParameter : children)
      {
         globalParameter.update(comment);
         globalParameter.updateChildren(comment);

         // notify change listener
      }
   }

   protected void update(String comment)
   {
      throw new RuntimeException("Must overwrite update");
   }

// protected abstract void update(String comment);
// {
//    throw new RuntimeException("Must overwrite update");
// }

   protected static void clearGlobalRegistry()
   {
      registry.clear();
   }
}
