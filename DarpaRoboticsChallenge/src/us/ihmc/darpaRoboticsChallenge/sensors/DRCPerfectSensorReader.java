package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.LinkedHashMap;
import java.util.Map.Entry;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.simulatedSensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RawSensorReader;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class DRCPerfectSensorReader implements SensorReader, RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private RawSensorReader rawSensorReader;
   private ControlFlowElement controllerDispatcher;

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private ForceSensorDataHolder forceSensorDataHolder;
   
   public DRCPerfectSensorReader()
   {
      
   }

   public void setSensorReader(RawSensorReader rawSensorReader)
   {
      this.rawSensorReader = rawSensorReader;
   }
   
   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
   }

   public void setControllerDispatcher(ControlFlowElement controllerDispatcher)
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

   public void doControl()
   {
      if(controllerDispatcher != null)
      {
         controllerDispatcher.waitUntilComputationIsDone();
      }
      
      read();
      
      if(controllerDispatcher != null)
      {
         controllerDispatcher.startComputation();
      }
   }

   private void read()
   {
      if(rawSensorReader != null)
      {
         rawSensorReader.read();
      }
      
      if(forceSensorDataHolder != null)
      {
         for(Entry<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensorEntry : forceTorqueSensors.entrySet())
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensorEntry.getValue();
            forceTorqueSensor.startComputation();
            forceTorqueSensor.waitUntilComputationIsDone();  
            forceSensorDataHolder.setForceSensorValue(forceTorqueSensorEntry.getKey(), forceTorqueSensor.getForceSensorOutputPort().getData());
         }
      }
   }
   
   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.put(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }

}
