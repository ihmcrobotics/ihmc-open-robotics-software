package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitionsFromRobotFactory;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class DRCPerfectSensorReaderFactory implements SensorReaderFactory
{
   private final SDFRobot robot;
   
   private final ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators;
   
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private final DRCPerfectSensorReader perfectSensorReader;
   
   public DRCPerfectSensorReaderFactory(SDFRobot robot, ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators, double estimateDT)
   {
      this.robot = robot;
      this.perfectSensorReader = new DRCPerfectSensorReader(estimateDT);
      this.groundContactPointBasedWrenchCalculators = groundContactPointBasedWrenchCalculators;
   }
   
   public void build(SixDoFJoint rootJoint, IMUDefinition[] imuDefinitions, boolean addLinearAccelerationSensors, YoVariableRegistry parentRegistry)
   {
      final Joint scsRootJoint = robot.getRootJoints().get(0);
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) scsRootJoint, rootJoint);

      StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(
            scsToInverseDynamicsJointMap, new ArrayList<IMUMount>(), groundContactPointBasedWrenchCalculators, addLinearAccelerationSensors);
      this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();
      
      
      SDFPerfectSimulatedSensorReader sdfPerfectSimulatedSensorReader = new SDFPerfectSimulatedSensorReader(robot, rootJoint, null);
      this.perfectSensorReader.setSensorReader(sdfPerfectSimulatedSensorReader);
      
      Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory
            .getForceSensorDefinitions();
      
      createAndAddForceSensors(forceSensorDefinitions);
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

   
   private void createAndAddForceSensors(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions2)
   {
      for(Entry<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionEntry : forceSensorDefinitions2.entrySet())
      {         
         WrenchCalculatorInterface groundContactPointBasedWrenchCalculator = forceSensorDefinitionEntry.getKey();
         perfectSensorReader.addForceTorqueSensorPort(forceSensorDefinitionEntry.getValue(), groundContactPointBasedWrenchCalculator);
      }
   }
}
