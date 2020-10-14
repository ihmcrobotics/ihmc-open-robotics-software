package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.parameters.ParameterData;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.variable.YoVariable;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class SaveableModuleState
{
   private final List<YoParameter> yoParameters = new ArrayList<>();
   private final List<YoVariable> yoVariables = new ArrayList<>();

   public void registerVariableToSave(YoVariable yoDouble)
   {
      yoVariables.add(yoDouble);
   }

   public void registerParameterToSave(YoParameter doubleParameter)
   {
      yoParameters.add(doubleParameter);
   }

   public void registerStateToSave(SaveableModuleState other)
   {
      for (int i = 0; i < other.getParametersToSave().size(); i++)
         yoParameters.add(other.getParametersToSave().get(i));
      for (int i = 0; i < other.getVariablesToSave().size(); i++)
         yoVariables.add(other.getVariablesToSave().get(i));
   }

   public List<YoParameter> getParametersToSave()
   {
      return yoParameters;
   }

   public List<YoVariable> getVariablesToSave()
   {
      return yoVariables;
   }

   public void loadValues(Map<String, ParameterData> dataMap) throws NoSuchFieldException, IllegalAccessException
   {
      Map<String, ParameterData> localMap = new HashMap<>(dataMap);

      for (int i = 0; i < yoParameters.size(); i++)
      {
         YoParameter parameter = yoParameters.get(i);
         ParameterData data = localMap.remove(parameter.getName());

         if (data != null)
            data.setParameterFromThis(parameter);
      }
      for (int i = 0; i < yoVariables.size(); i++)
      {
         YoVariable variable = yoVariables.get(i);
         ParameterData data = localMap.remove(variable.getName());

         if (data != null)
         {
            Field valueField = ParameterData.class.getField("value");
            valueField.setAccessible(true);
            String value = (String) valueField.get(data);
            variable.parseValue(value);
         }
      }
   }
}
