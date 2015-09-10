package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;

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
   public void read()
   {
      if(rawSensorReader != null)
      {
         rawSensorReader.read();
      }
   }

   @Override
   public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
