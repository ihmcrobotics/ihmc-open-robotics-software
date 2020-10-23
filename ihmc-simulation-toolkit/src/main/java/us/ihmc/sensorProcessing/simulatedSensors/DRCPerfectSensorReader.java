package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DRCPerfectSensorReader implements SensorReader
{
   private final YoRegistry registry = new YoRegistry("DRCPerfectSensorReader");
   private RawSensorReader rawSensorReader;
   private SensorOutputMapReadOnly processedSensorOutputMap;
   private SensorOutputMapReadOnly rawSensorOutputMap;

   public DRCPerfectSensorReader(double estimateDT)
   {
   }

   public void setSensorReader(RawSensorReader rawSensorReader)
   {
      this.rawSensorReader = rawSensorReader;
   }

   public void setProcessedSensorOutputMap(SensorOutputMapReadOnly processedSensorOutputMap)
   {
      this.processedSensorOutputMap = processedSensorOutputMap;
   }

   public void setRawSensorOutputMap(SensorOutputMapReadOnly rawSensorOutputMap)
   {
      this.rawSensorOutputMap = rawSensorOutputMap;
   }

   @Override
   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      return processedSensorOutputMap;
   }

   @Override
   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      return rawSensorOutputMap;
   }

   public YoRegistry getYoVariableRegistry()
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
      return processedSensorOutputMap.getMonotonicTime();
   }
}
