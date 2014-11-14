package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.LinkedHashMap;
import java.util.Map.Entry;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.ControllerDispatcher;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class DRCPerfectSensorReader implements SensorReader, RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private final IntegerYoVariable step = new IntegerYoVariable("step", registry);
   private final long estimateDTinNs;
   private RawSensorReader rawSensorReader;
   private ControllerDispatcher controllerDispatcher;

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private ForceSensorDataHolder forceSensorDataHolder;
   
   public DRCPerfectSensorReader(double estimateDT)
   {
      this.estimateDTinNs = TimeTools.secondsToNanoSeconds(estimateDT);
      step.set(9807);
   }

   public void setSensorReader(RawSensorReader rawSensorReader)
   {
      this.rawSensorReader = rawSensorReader;
   }

   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      throw new RuntimeException("Should not get there");
   }

   public void setControllerDispatcher(ControllerDispatcher controllerDispatcher)
   {
      this.controllerDispatcher = controllerDispatcher;
   }

   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder)
   {
      this.forceSensorDataHolder = forceSensorDataHolder;
   }

   public void initialize()
   {
      read();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "DRCPerfectSensorReader";
   }

   public String getDescription()
   {
      return getName();
   }

   @Deprecated
   public void doControl()
   {
      read();
   }

   public void read()
   {
      step.increment();
      if(rawSensorReader != null)
      {
         rawSensorReader.read();
      }
      
      if(forceSensorDataHolder != null)
      {
         for(Entry<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensorEntry : forceTorqueSensors.entrySet())
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensorEntry.getValue();
            forceTorqueSensor.calculate();  
            forceSensorDataHolder.setForceSensorValue(forceTorqueSensorEntry.getKey(), forceTorqueSensor.getWrench());
         }
      }
      
      if(controllerDispatcher != null)
      {
         controllerDispatcher.startEstimator(estimateDTinNs * step.getIntegerValue(), System.nanoTime());
      }
   }
   
   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.put(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }
}
