package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitionsFromRobotFactory;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DRCPerfectSensorReaderFactory implements SensorReaderFactory
{
   private final SDFRobot robot;
 
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private final DRCPerfectSensorReader perfectSensorReader;
   
   public DRCPerfectSensorReaderFactory(SDFRobot robot, double estimateDT)
   {
      this.robot = robot;
      this.perfectSensorReader = new DRCPerfectSensorReader(estimateDT);
   }
   
   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ForceSensorDataHolder forceSensorDataHolderForEstimator, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry parentRegistry)
   {
      final Joint scsRootJoint = robot.getRootJoints().get(0);
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) scsRootJoint, rootJoint);

      StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(
            scsToInverseDynamicsJointMap, new ArrayList<IMUMount>(), new ArrayList<WrenchCalculatorInterface>());
      this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();
      
      
      SDFPerfectSimulatedSensorReader sdfPerfectSimulatedSensorReader = new SDFPerfectSimulatedSensorReader(robot, rootJoint, null);
      this.perfectSensorReader.setSensorReader(sdfPerfectSimulatedSensorReader);
      this.perfectSensorReader.setForceSensorDataHolder(forceSensorDataHolderForEstimator);
      
      Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensors = stateEstimatorSensorDefinitionsFromRobotFactory
            .getForceSensorDefinitions();
      
      createAndAddForceSensors(forceSensors);
   }

   public DRCPerfectSensorReader getSensorReader()
   {
      return perfectSensorReader;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public boolean useStateEstimator()
   {
      return false;
   }

   
   private void createAndAddForceSensors(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensors)
   {
      for(Entry<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionEntry : forceSensors.entrySet())
      {         
         WrenchCalculatorInterface groundContactPointBasedWrenchCalculator = forceSensorDefinitionEntry.getKey();
         perfectSensorReader.addForceTorqueSensorPort(forceSensorDefinitionEntry.getValue(), groundContactPointBasedWrenchCalculator);
      }
   }
}
