package us.ihmc.robotics.sensors;

import java.util.List;

public interface ForceSensorDataHolderReadOnly
{
   default ForceSensorDataReadOnly getData(ForceSensorDefinition sensorDefinition)
   {
      return getData(sensorDefinition.getSensorName());
   }

   ForceSensorDataReadOnly getData(String sensorName);

   ForceSensorDefinition getDefinition(String sensorName);

   List<ForceSensorDefinition> getForceSensorDefinitions();
}
