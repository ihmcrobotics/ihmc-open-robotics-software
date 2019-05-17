package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DRCPerfectSensorReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private RawSensorReader rawSensorReader;
   private SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;

   public DRCPerfectSensorReader(double estimateDT)
   {
   }

   public void setSensorReader(RawSensorReader rawSensorReader)
   {
      this.rawSensorReader = rawSensorReader;
   }

   public void setSensorOutputMapReadOnly(SensorOutputMapReadOnly sensorOutputMapReadOnly)
   {
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
   }

   public void setSensorRawOutputMapReadOnly(SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly)
   {
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorOutputMapReadOnly;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorRawOutputMapReadOnly;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public long read(SensorDataContext sensorDataContext)
   {
      if (rawSensorReader != null)
      {
         rawSensorReader.read();
      }
      return sensorOutputMapReadOnly.getTimestamp();
   }
}
