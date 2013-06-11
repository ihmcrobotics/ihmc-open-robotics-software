package us.ihmc.darpaRoboticsChallenge.sensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.sensorProcessing.simulatedSensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitionsFromRobotFactory;
import us.ihmc.sensorProcessing.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.KinematicPoint;

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
   
   public void build(SixDoFJoint sixDoFJoint, IMUDefinition[] imuDefinitions, boolean addLinearAccelerationSensors)
   {
      final Joint rootJoint = robot.getRootJoints().get(0);
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) rootJoint, sixDoFJoint);

      StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(
            scsToInverseDynamicsJointMap, robot, new ArrayList<IMUMount>(), groundContactPointBasedWrenchCalculators,
            new ArrayList<KinematicPoint>(), new ArrayList<KinematicPoint>(), addLinearAccelerationSensors);
      this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();
      
      
      SDFPerfectSimulatedSensorReader sdfPerfectSimulatedSensorReader = new SDFPerfectSimulatedSensorReader(robot, sixDoFJoint, null);
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
