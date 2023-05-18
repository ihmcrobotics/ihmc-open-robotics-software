package us.ihmc.robotics.sensors;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface ForceSensorDataHolderReadOnly
{
   ForceSensorDataReadOnly get(ForceSensorDefinition forceSensor);

   default ForceSensorDataReadOnly getByName(String sensorName)
   {
      ForceSensorDefinition forceSensorDefinition = findForceSensorDefinition(sensorName);

      if (forceSensorDefinition == null)
         throw new RuntimeException("Force sensor not found " + sensorName);
      else
         return get(forceSensorDefinition);
   }

   ForceSensorDefinition findForceSensorDefinition(String name);

   List<ForceSensorDefinition> getForceSensorDefinitions();

   default WrenchReadOnly getForceSensorValue(ForceSensorDefinition key)
   {
      return get(key).getWrench();
   }

   default DMatrixRMaj getForceSensorMatrixValue(ForceSensorDefinition key)
   {
      return get(key).getWrenchMatrix();
   }

}
