package us.ihmc.robotics.saveableModule;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class SaveableModuleState
{
   private final List<YoDouble> doubles = new ArrayList<>();
   private final List<YoInteger> integers = new ArrayList<>();
   private final List<YoBoolean> booleans = new ArrayList<>();

   public void registerDoubleToSave(YoDouble yoDouble)
   {
      doubles.add(yoDouble);
   }

   public void registerIntegerToSave(YoInteger yoInteger)
   {
      integers.add(yoInteger);
   }

   public void registerBooleanToSave(YoBoolean yoBoolean)
   {
      booleans.add(yoBoolean);
   }

   public void registerStateToSave(SaveableModuleState other)
   {
      for (int i = 0; i < other.getDoublesToSave().size(); i++)
         doubles.add(other.getDoublesToSave().get(i));
      for (int i = 0; i < other.getIntegersToSave().size(); i++)
         integers.add(other.getIntegersToSave().get(i));
      for (int i = 0; i < other.getBooleansToSave().size(); i++)
         booleans.add(other.getBooleansToSave().get(i));
   }

   public List<YoDouble> getDoublesToSave()
   {
      return doubles;
   }

   public List<YoInteger> getIntegersToSave()
   {
      return integers;
   }

   public List<YoBoolean> getBooleansToSave()
   {
      return booleans;
   }

   @Override
   public String toString()
   {
      String string = "";
      for (int i = 0; i < doubles.size(); i++)
      {
         YoDouble yoDouble = doubles.get(i);
         string += yoDouble.getFullNameString() + ": " + yoDouble.getValue() + ", ";
      }
      for (int i = 0; i < integers.size(); i++)
      {
         YoInteger yoInteger = integers.get(i);
         string += yoInteger.getFullNameString() + ": " + yoInteger.getValue() + ", ";
      }
      for (int i = 0; i < booleans.size(); i++)
      {
         YoBoolean yoBoolean = booleans.get(i);
         string += yoBoolean.getFullNameString() + ": " + yoBoolean.getValue() + ", ";
      }

      return string;
   }

   public void loadValues(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      for (int i = 0; i < doubles.size(); i++)
         doubles.get(i).set(SaveableModuleTools.readNextDouble(scanner));
      for (int i = 0; i < integers.size(); i++)
         integers.get(i).set(SaveableModuleTools.readNextInt(scanner));
      for (int i = 0; i < booleans.size(); i++)
         booleans.get(i).set(SaveableModuleTools.readNextBoolean(scanner));
      scanner.close();
   }
}
