package us.ihmc.steppr.hardware.sensorReader;

import java.io.IOException;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.acsell.hardware.sensorReader.AcsellSensorReader;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.state.StepprState;

public class StepprSensorReaderFactory implements SensorReaderFactory
{
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private AcsellSensorReader<StepprJoint> sensorReader;

   private final DRCRobotSensorInformation sensorInformation;
   private final StateEstimatorParameters stateEstimatorParameters;
   
   public StepprSensorReaderFactory(DRCRobotModel robotModel)
   {
      sensorInformation = robotModel.getSensorInformation();
      stateEstimatorParameters = robotModel.getStateEstimatorParameters();
   }

   @Override
   public void build(FloatingInverseDynamicsJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
         ContactSensorHolder contactSensorHolder, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, DesiredJointDataHolder estimatorDesiredJointDataHolder, YoVariableRegistry parentRegistry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      YoVariableRegistry sensorReaderRegistry = new YoVariableRegistry("StepprSensorReader");
      StepprState state = new StepprState(stateEstimatorParameters.getEstimatorDT(), sensorReaderRegistry);
      List<OneDoFJoint> jointList = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      EnumMap<StepprJoint, OneDoFJoint> stepprJoints = StepprUtil.createJointMap(jointList);
      sensorReader = new AcsellSensorReader<StepprJoint>(state, StepprJoint.values, stepprJoints, stateEstimatorSensorDefinitions, rawJointSensorDataHolderMap, sensorInformation, stateEstimatorParameters,
            estimatorDesiredJointDataHolder, sensorReaderRegistry);
      
      
      parentRegistry.addChild(sensorReaderRegistry);
   }

   @Override
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public void connect() throws IOException
   {
      sensorReader.connect();
   }

   public void disconnect()
   {
      sensorReader.disconnect();
   }

   @Override
   public boolean useStateEstimator()
   {
      return true;
   }

}
