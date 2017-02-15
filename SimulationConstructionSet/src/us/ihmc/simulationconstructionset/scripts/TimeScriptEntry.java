package us.ihmc.simulationconstructionset.scripts;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class TimeScriptEntry implements Comparable<Object>
{
   private double time;
   private ArrayList<VariableValuePair> varValuePairs;
   private ArrayList<TimeScriptCommand> timeScriptCommands;
   
   public TimeScriptEntry(double time)
   {
      this.time = time;
      varValuePairs = new ArrayList<VariableValuePair>();
      timeScriptCommands = new ArrayList<TimeScriptCommand>();
   }

   public void addVarValue(DoubleYoVariable variable, double value)
   {
      if (variable == null)
         return;
      VariableValuePair variableValuePair = new DoubleVariableValuePair(variable, value);
      varValuePairs.add(variableValuePair);
   }
   
   public void addVarValue(BooleanYoVariable variable, boolean value)
   {
      if (variable == null)
         return;
      VariableValuePair variableValuePair = new BooleanVariableValuePair(variable, value);
      varValuePairs.add(variableValuePair);
   }
   
   public void addVarValue(IntegerYoVariable variable, int value)
   {
      if (variable == null)
         return;
      VariableValuePair variableValuePair = new IntegerVariableValuePair(variable, value);
      varValuePairs.add(variableValuePair);
   }
   
   @SuppressWarnings("rawtypes")
   public void addVarValue(EnumYoVariable variable, Enum value)
   {
      if (variable == null)
         return;
      VariableValuePair variableValuePair = new EnumVariableValuePair(variable, value);
      varValuePairs.add(variableValuePair);
   }

   @Override
   public int compareTo(Object timeScriptEntry)
   {
      if (timeScriptEntry == this)
         return 0;
      if (((TimeScriptEntry) timeScriptEntry).time < time)
         return 1;
      else
         return -1;
   }

   public void setVarsToValues()
   {
      for (int i = 0; i < varValuePairs.size(); i++)
      {
         VariableValuePair variableValuePair = varValuePairs.get(i);
         variableValuePair.setVariableToValue();
      }
   }
   
   public void doCommands()
   {
      for (int i=0; i<timeScriptCommands.size(); i++)
      {
         TimeScriptCommand timeScriptCommand = timeScriptCommands.get(i);
         timeScriptCommand.doCommand();
      }
   }
   
   
   public void addTimeScriptCommand(TimeScriptCommand timeScriptCommand)
   {
      timeScriptCommands.add(timeScriptCommand);
   }

   public double getTime()
   {
      return this.time;
   }

   private interface VariableValuePair
   {
      public abstract void setVariableToValue();
   }
   
   private class DoubleVariableValuePair implements VariableValuePair
   {
      protected DoubleYoVariable variable;
      protected double value;

      public DoubleVariableValuePair(DoubleYoVariable variable, double value)
      {
         this.variable = variable;
         this.value = value;
      }

      @Override
      public void setVariableToValue()
      {
         variable.set(value);
      }
   }
   
   private class BooleanVariableValuePair implements VariableValuePair
   {
      protected BooleanYoVariable variable;
      protected boolean value;

      public BooleanVariableValuePair(BooleanYoVariable variable, boolean value)
      {
         this.variable = variable;
         this.value = value;
      }

      @Override
      public void setVariableToValue()
      {
         variable.set(value);
      }
   }
   
   private class IntegerVariableValuePair implements VariableValuePair
   {
      protected IntegerYoVariable variable;
      protected int value;

      public IntegerVariableValuePair(IntegerYoVariable variable, int value)
      {
         this.variable = variable;
         this.value = value;
      }

      @Override
      public void setVariableToValue()
      {
         variable.set(value);
      }
   }
   
   private class EnumVariableValuePair implements VariableValuePair
   {
      @SuppressWarnings("rawtypes")
      protected EnumYoVariable variable;
      @SuppressWarnings("rawtypes")
      protected Enum value;

      @SuppressWarnings("rawtypes")
      public EnumVariableValuePair(EnumYoVariable variable, Enum value)
      {
         this.variable = variable;
         this.value = value;
      }

      @Override
      @SuppressWarnings("unchecked")
      public void setVariableToValue()
      {
         variable.set(value);
      }
   }
 
   
}
