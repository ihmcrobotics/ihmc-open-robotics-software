package us.ihmc.robotics.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class ForceSensorDataHolder implements ForceSensorDataHolderReadOnly
{
   private final HashMap<ForceSensorDefinition, ForceSensorData> forceSensors = new HashMap<ForceSensorDefinition, ForceSensorData>();
   private final HashMap<String, ForceSensorDefinition> sensorNameToDefintionMap = new HashMap<>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();

   public ForceSensorDataHolder(List<ForceSensorDefinition> forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         ForceSensorData forceSensor = new ForceSensorData(forceSensorDefinition);
         forceSensorDefinitions.add(forceSensorDefinition);
         this.forceSensors.put(forceSensorDefinition, forceSensor);
         sensorNameToDefintionMap.put(forceSensorDefinition.getSensorName(), forceSensorDefinition);
      }
   }

   @Override
   public ForceSensorData get(ForceSensorDefinition forceSensor)
   {
      return forceSensors.get(forceSensor);
   }

   @Override
   public List<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   @Override
   public ForceSensorData getByName(String sensorName)
   {
      ForceSensorDefinition forceSensorDefinition = findForceSensorDefinition(sensorName);

      if (forceSensorDefinition == null)
         throw new RuntimeException("Force sensor not found " + sensorName);
      else
         return get(forceSensorDefinition);
   }

   @Override
   public ForceSensorDefinition findForceSensorDefinition(String sensorName)
   {
      return sensorNameToDefintionMap.get(sensorName);
   }

   private final DenseMatrix64F tempWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   public boolean firstException = true;

   public void set(ForceSensorDataHolderReadOnly otherForceSensorDataHolder)
   {
      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         final ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         ForceSensorDataReadOnly otherForceSensorData = otherForceSensorDataHolder.get(forceSensorDefinition);
         if (otherForceSensorData == null)
          {
             if (firstException)
             {
                firstException = false;
               System.err.println(getClass().getSimpleName() + " Could not find the force sensor: " + forceSensorDefinition.getSensorName());
             }
          }
          else
          {
             otherForceSensorData.getWrench(tempWrench);
             forceSensors.get(forceSensorDefinition).setWrench(tempWrench);
          }
      }
   }

   public void setForceSensorValue(ForceSensorDefinition key, DenseMatrix64F data)
   {
      forceSensors.get(key).setWrench(data);
   }

   public void setForceSensorValue(ForceSensorDefinition key, WrenchReadOnly wrench)
   {
      forceSensors.get(key).setWrench(wrench);
   }

   @Override
   public void getForceSensorValue(ForceSensorDefinition key, Wrench wrenchToPack)
   {
      forceSensors.get(key).getWrench(wrenchToPack);
   }

   @Override
   public void getForceSensorValue(ForceSensorDefinition key, DenseMatrix64F wrenchToPack)
   {
      forceSensors.get(key).getWrench(wrenchToPack);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         final ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         forceSensors.get(forceSensorDefinition).calculateChecksum(checksum);
      }
   }
}
