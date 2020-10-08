package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public abstract class SaveableModuleState
{
   private final List<YoDouble> doubles = new ArrayList<>();
   private final List<YoInteger> integers = new ArrayList<>();
   private final List<YoBoolean> booleans = new ArrayList<>();
   private final List<YoEnum<?>> enums = new ArrayList<>();

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

   public void registerEnumToSave(YoEnum<?> yoEnum)
   {
      enums.add(yoEnum);
   }

   public void registerStateToSave(SaveableModuleState other)
   {
      for (int i = 0; i < other.getDoublesToSave().size(); i++)
         doubles.add(other.getDoublesToSave().get(i));
      for (int i = 0; i < other.getIntegersToSave().size(); i++)
         integers.add(other.getIntegersToSave().get(i));
      for (int i = 0; i < other.getBooleansToSave().size(); i++)
         booleans.add(other.getBooleansToSave().get(i));
      for (int i = 0; i < other.getEnumsToSave().size(); i++)
         enums.add(other.getEnumsToSave().get(i));
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

   public List<YoEnum<?>> getEnumsToSave()
   {
      return enums;
   }

   @Override
   public String toString()
   {
      String string = "";
      for (int i = 0; i < doubles.size(); i++)
      {
         YoDouble yoDouble = doubles.get(i);
         string += "\n" + yoDouble.getName();
         string += "\n" + yoDouble.getValue();
      }
      for (int i = 0; i < integers.size(); i++)
      {
         YoInteger yoInteger = integers.get(i);
         string += "\n" + yoInteger.getName();
         string += "\n" + yoInteger.getValue();
      }
      for (int i = 0; i < booleans.size(); i++)
      {
         YoBoolean yoBoolean = booleans.get(i);
         string += "\n" + yoBoolean.getName();
         string += "\n" + yoBoolean.getValue();
      }
      for (int i = 0; i < enums.size(); i++)
      {
         YoEnum<?> yoEnum = enums.get(i);
         string += "\n" + yoEnum.getName();
         string += "\n" + yoEnum.getOrdinal();
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
      for (int i = 0; i < enums.size(); i++)
         enums.get(i).set(SaveableModuleTools.readNextInt(scanner));

      scanner.close();
   }
}
