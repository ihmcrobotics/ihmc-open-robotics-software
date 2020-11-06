package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.parameters.ParameterData;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.variable.YoVariable;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class YoSaveableModuleState
{
   private final List<YoParameter> yoParameters = new ArrayList<>();
   private final List<YoVariable> yoVariables = new ArrayList<>();

   public void registerVariableToSave(YoVariable yoDouble)
   {
      yoVariables.add(yoDouble);
   }

   public void registerVariableToSave(YoParameter doubleParameter)
   {
      yoParameters.add(doubleParameter);
   }

   public void registerStateToSave(YoSaveableModuleState other)
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

   public void loadValues(Map<String, ParameterData> dataMap)
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
            Field valueField;
            try
            {
               valueField = ParameterData.class.getDeclaredField("value");
            }
            catch (NoSuchFieldException e)
            {
               throw new RuntimeException("This should never happen.");
            }
            valueField.setAccessible(true);
            String value;
            try
            {
               value  = (String) valueField.get(data);
            }
            catch (IllegalAccessException e)
            {
               throw new RuntimeException("This also should never happen.");
            }
            variable.parseValue(value);
         }
      }
   }
}
