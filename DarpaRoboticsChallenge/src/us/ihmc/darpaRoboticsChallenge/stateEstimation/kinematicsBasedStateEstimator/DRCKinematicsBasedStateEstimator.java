package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SimplePositionStateCalculatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.ControlFlowGraphExecutorController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.SensorAndEstimatorAssembler;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface
{
   //   private final SensorNoiseParameters sensorNoiseParametersForEstimator =
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SensorReader sensorReader;

   private final SensorAndEstimatorAssembler sensorAndEstimatorAssembler;
   private final StateEstimatorWithPorts stateEstimatorWithPorts;
   private final ControlFlowGraphExecutorController controlFlowGraphExecutorController;

   //      DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   private final SensorNoiseParameters sensorNoiseParametersForEstimator = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();
   
   private final SimplePositionStateCalculatorInterface simplePositionStateRobotModelUpdater;
   
   public DRCKinematicsBasedStateEstimator(StateEstimationDataFromController stateEstimatorDataFromControllerSource,
         FullInverseDynamicsStructure inverseDynamicsStructure, AfterJointReferenceFrameNameMap estimatorReferenceFrameMap,
         RigidBodyToIndexMap estimatorRigidBodyToIndexMap, double estimateDT, SensorReaderFactory sensorReaderFactory, double gravitationalAcceleration,
         SideDependentList<WrenchBasedFootSwitch> footSwitches, SideDependentList<ContactablePlaneBody> bipedFeet,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      sensorReader = sensorReaderFactory.getSensorReader();

      SensorFilterParameters sensorFilterParameters = new SensorFilterParameters(
            DRCConfigParameters.JOINT_POSITION_FILTER_FREQ_HZ, DRCConfigParameters.JOINT_VELOCITY_FILTER_FREQ_HZ, 
            DRCConfigParameters.ORIENTATION_FILTER_FREQ_HZ, DRCConfigParameters.ANGULAR_VELOCITY_FILTER_FREQ_HZ,
            DRCConfigParameters.LINEAR_ACCELERATION_FILTER_FREQ_HZ, estimateDT);
      
      PointMeasurementNoiseParameters pointMeasurementNoiseParameters = new  PointMeasurementNoiseParameters(
            DRCConfigParameters.pointVelocityXYMeasurementStandardDeviation,
            DRCConfigParameters.pointVelocityZMeasurementStandardDeviation,
            DRCConfigParameters.pointPositionXYMeasurementStandardDeviation,
            DRCConfigParameters.pointPositionZMeasurementStandardDeviation);

      // That thing is ugly, I'm sorry!! (Sylvain)
      simplePositionStateRobotModelUpdater = new PelvisStateCalculator(inverseDynamicsStructure, footSwitches, bipedFeet, gravitationalAcceleration,
            estimateDT, dynamicGraphicObjectsListRegistry, registry);

      // Make the estimator here.
      sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(stateEstimatorDataFromControllerSource,
            sensorReaderFactory.getStateEstimatorSensorDefinitions(), sensorNoiseParametersForEstimator, sensorFilterParameters,
            pointMeasurementNoiseParameters, gravitationalAcceleration, inverseDynamicsStructure, estimatorReferenceFrameMap, estimatorRigidBodyToIndexMap,
            estimateDT, true, true, simplePositionStateRobotModelUpdater, registry);

      stateEstimatorWithPorts = sensorAndEstimatorAssembler.getEstimator();

      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = sensorAndEstimatorAssembler.getJointAndIMUSensorDataSource();
      sensorReader.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);
      
      if (simplePositionStateRobotModelUpdater != null)
      {
         simplePositionStateRobotModelUpdater.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);
      }
      
      ControlFlowGraph controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();

      controlFlowGraphExecutorController = new ControlFlowGraphExecutorController(controlFlowGraph);
      registry.addChild(controlFlowGraphExecutorController.getYoVariableRegistry());
   }

   public StateEstimator getStateEstimator()
   {
      return stateEstimatorWithPorts;
   }

   public void initialize()
   {
      sensorAndEstimatorAssembler.initialize();
      doControl();
   }
   
   public void startIMUDriftEstimation()
   {
      simplePositionStateRobotModelUpdater.startIMUDriftEstimation();
   }

   public void startIMUDriftCompensation()
   {
      simplePositionStateRobotModelUpdater.startIMUDriftCompensation();
   }
   
   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      // Setting the initial CoM Position here.
      FramePoint estimatedCoMPosition = new FramePoint();
      stateEstimatorWithPorts.getEstimatedCoMPosition(estimatedCoMPosition);
      estimatedCoMPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      estimatedCoMPosition.set(initialCoMPosition);

      FrameOrientation estimatedOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), initialEstimationLinkOrientation);

      sensorAndEstimatorAssembler.initializeEstimatorToActual(estimatedCoMPosition, estimatedOrientation);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      controlFlowGraphExecutorController.doControl();
   }

   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolderForEstimator)
   {
      sensorReader.setForceSensorDataHolder(forceSensorDataHolderForEstimator);
   }
}
