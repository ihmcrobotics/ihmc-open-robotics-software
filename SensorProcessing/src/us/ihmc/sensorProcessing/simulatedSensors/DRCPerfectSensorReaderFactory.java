package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
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
      perfectSensorReader = new DRCPerfectSensorReader(estimateDT);
   }

   @Override
   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ForceSensorDataHolder forceSensorDataHolderForEstimator, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry parentRegistry)
   {
      final Joint scsRootJoint = robot.getRootJoints().get(0);
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) scsRootJoint, rootJoint);

      ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCaclculators = new ArrayList<WrenchCalculatorInterface>();
      robot.getForceSensors(groundContactPointBasedWrenchCaclculators);
      StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(
            scsToInverseDynamicsJointMap, new ArrayList<IMUMount>(), groundContactPointBasedWrenchCaclculators);
      stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();

      SDFPerfectSimulatedSensorReader sdfPerfectSimulatedSensorReader = new SDFPerfectSimulatedSensorReader(robot, rootJoint, null);
      perfectSensorReader.setSensorReader(sdfPerfectSimulatedSensorReader);
      perfectSensorReader.setSensorOutputMapReadOnly(sdfPerfectSimulatedSensorReader);
      perfectSensorReader.setForceSensorDataHolder(forceSensorDataHolderForEstimator);

      Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensors = stateEstimatorSensorDefinitionsFromRobotFactory.getForceSensorDefinitions();

      createAndAddForceSensors(forceSensors);
   }

   @Override
   public DRCPerfectSensorReader getSensorReader()
   {
      return perfectSensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return false;
   }

   private void createAndAddForceSensors(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensors)
   {
      for (Entry<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionEntry : forceSensors.entrySet())
      {
         WrenchCalculatorInterface groundContactPointBasedWrenchCalculator = forceSensorDefinitionEntry.getKey();
         perfectSensorReader.addForceTorqueSensorPort(forceSensorDefinitionEntry.getValue(), groundContactPointBasedWrenchCalculator);
      }
   }
}
