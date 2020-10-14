package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

// TODO add support for parameters

public abstract class SaveableModuleState
{
   private final List<YoDouble> yoDoubles = new ArrayList<>();
   private final List<YoInteger> yoIntegers = new ArrayList<>();
   private final List<YoBoolean> yoBooleans = new ArrayList<>();
   private final List<YoEnum<?>> yoEnums = new ArrayList<>();

   private final List<DoubleParameter> doubleParameters = new ArrayList<>();
   private final List<IntegerParameter> integerParameters = new ArrayList<>();
   private final List<BooleanParameter> booleanParameters = new ArrayList<>();
   private final List<EnumParameter<?>> enumParameters = new ArrayList<>();

   public void registerDoubleToSave(YoDouble yoDouble)
   {
      yoDoubles.add(yoDouble);
   }

   public void registerDoubleToSave(DoubleParameter doubleParameter)
   {
      doubleParameters.add(doubleParameter);
   }

   public void registerIntegerToSave(YoInteger yoInteger)
   {
      yoIntegers.add(yoInteger);
   }

   public void registerIntegerToSave(IntegerParameter integerPara)
   {
      integerParameters.add(integerPara);
   }

   public void registerBooleanToSave(YoBoolean yoBoolean)
   {
      yoBooleans.add(yoBoolean);
   }

   public void registerBooleanToSave(BooleanParameter booleanParameter)
   {
      booleanParameters.add(booleanParameter);
   }

   public void registerEnumToSave(YoEnum<?> yoEnum)
   {
      yoEnums.add(yoEnum);
   }

   public void registerEnumToSave(EnumParameter<?> enumParameter)
   {
      enumParameters.add(enumParameter);
   }
   public void registerStateToSave(SaveableModuleState other)
   {
      for (int i = 0; i < other.getYoDoublesToSave().size(); i++)
         yoDoubles.add(other.getYoDoublesToSave().get(i));
      for (int i = 0; i < other.getYoIntegersToSave().size(); i++)
         yoIntegers.add(other.getYoIntegersToSave().get(i));
      for (int i = 0; i < other.getYoBooleansToSave().size(); i++)
         yoBooleans.add(other.getYoBooleansToSave().get(i));
      for (int i = 0; i < other.getYoEnumsToSave().size(); i++)
         yoEnums.add(other.getYoEnumsToSave().get(i));
   }

   private List<YoDouble> getYoDoublesToSave()
   {
      return yoDoubles;
   }

   private List<DoubleParameter> getDoubleParametersToSave()
   {
      return doubleParameters;
   }

   private List<YoInteger> getYoIntegersToSave()
   {
      return yoIntegers;
   }

   private List<IntegerParameter> getIntegerParametersToSave()
   {
      return integerParameters;
   }

   private List<YoBoolean> getYoBooleansToSave()
   {
      return yoBooleans;
   }

   private List<BooleanParameter> getBooleanParametersToSave()
   {
      return booleanParameters;
   }

   private List<YoEnum<?>> getYoEnumsToSave()
   {
      return yoEnums;
   }

   private List<EnumParameter<?>> getEnumParametersToSave()
   {
      return enumParameters;
   }

   @Override
   public String toString()
   {
      String string = "";
      for (int i = 0; i < yoDoubles.size(); i++)
      {
         YoDouble yoDouble = yoDoubles.get(i);
         string += "\n" + yoDouble.getName();
         string += "\n" + yoDouble.getValue();
      }
      for (int i = 0; i < yoIntegers.size(); i++)
      {
         YoInteger yoInteger = yoIntegers.get(i);
         string += "\n" + yoInteger.getName();
         string += "\n" + yoInteger.getValue();
      }
      for (int i = 0; i < yoBooleans.size(); i++)
      {
         YoBoolean yoBoolean = yoBooleans.get(i);
         string += "\n" + yoBoolean.getName();
         string += "\n" + yoBoolean.getValue();
      }
      for (int i = 0; i < yoEnums.size(); i++)
      {
         YoEnum<?> yoEnum = yoEnums.get(i);
         string += "\n" + yoEnum.getName();
         string += "\n" + yoEnum.getOrdinal();
      }

      return string;
   }

   public void loadValues(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      for (int i = 0; i < yoDoubles.size(); i++)
         yoDoubles.get(i).set(SaveableModuleTools.readNextDouble(scanner));
      for (int i = 0; i < yoIntegers.size(); i++)
         yoIntegers.get(i).set(SaveableModuleTools.readNextInt(scanner));
      for (int i = 0; i < yoBooleans.size(); i++)
         yoBooleans.get(i).set(SaveableModuleTools.readNextBoolean(scanner));
      for (int i = 0; i < yoEnums.size(); i++)
         yoEnums.get(i).set(SaveableModuleTools.readNextInt(scanner));

      scanner.close();
   }
}
