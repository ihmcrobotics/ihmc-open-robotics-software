package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.StateEstimatorParameters;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable yoTime = new DoubleYoVariable("t_stateEstimator", registry);
   
   private final SensorReader sensorReader;

   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdater pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
 
   private final double estimatorDT;
   
   private boolean visualize = false;
   private final ArrayList<DynamicGraphicReferenceFrame> dynamicGraphicMeasurementFrames = new ArrayList<>();
   
   private final SensorNoiseParameters sensorNoiseParametersForEstimator = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();
   
   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, RigidBodyToIndexMap estimatorRigidBodyToIndexMap,
         double estimatorDT, StateEstimatorParameters stateEstimatorParameters, SensorReaderFactory sensorReaderFactory, double gravitationalAcceleration,
         SideDependentList<WrenchBasedFootSwitch> footSwitches, SideDependentList<ContactablePlaneBody> bipedFeet,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.estimatorDT = estimatorDT;
      
      sensorReader = sensorReaderFactory.getSensorReader();

      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = new JointAndIMUSensorDataSource(sensorReaderFactory.getStateEstimatorSensorDefinitions(), stateEstimatorParameters.getSensorFilterParameters(estimatorDT), registry);
      JointAndIMUSensorMap jointAndIMUSensorMap = jointAndIMUSensorDataSource.getSensorMap();

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, jointAndIMUSensorDataSource, registry);

      SensorConfigurationFactory sensorConfigurationFactory = new SensorConfigurationFactory(sensorNoiseParametersForEstimator, gravitationalAcceleration);
      
      List<OrientationSensorConfiguration> orientationSensorConfigurations = sensorConfigurationFactory.createOrientationSensorConfigurations(jointAndIMUSensorMap.getOrientationSensors());
      List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = sensorConfigurationFactory.createAngularVelocitySensorConfigurations(jointAndIMUSensorMap.getAngularVelocitySensors());
      List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = sensorConfigurationFactory.createLinearAccelerationSensorConfigurations(jointAndIMUSensorMap.getLinearAccelerationSensors());
      
      pelvisRotationalStateUpdater = new PelvisRotationalStateUpdater(inverseDynamicsStructure, orientationSensorConfigurations, angularVelocitySensorConfigurations, registry);

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, angularVelocitySensorConfigurations, linearAccelerationSensorConfigurations, footSwitches, bipedFeet,
            gravitationalAcceleration, yoTime, estimatorDT, dynamicGraphicObjectsListRegistry, registry);

      sensorReader.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);
      
      visualize = visualize && dynamicGraphicObjectsListRegistry != null;
      
      if (visualize)
         setupDynamicGraphicObjects(dynamicGraphicObjectsListRegistry, orientationSensorConfigurations);
   }

   private void setupDynamicGraphicObjects(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         List<OrientationSensorConfiguration> orientationSensorConfigurations)
   {
      for (int i = 0; i < orientationSensorConfigurations.size(); i++)
      {
         DynamicGraphicReferenceFrame dynamicGraphicMeasurementFrame = new DynamicGraphicReferenceFrame(orientationSensorConfigurations.get(i).getMeasurementFrame(), registry, 1.0);
         dynamicGraphicMeasurementFrames.add(dynamicGraphicMeasurementFrame);
      }
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("imuFrame", dynamicGraphicMeasurementFrames);
   }

   public StateEstimator getStateEstimator()
   {
      return this;
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      pelvisRotationalStateUpdater.initialize();
      pelvisLinearStateUpdater.initialize();
   }
   
   public void doControl()
   {
      yoTime.add(estimatorDT); //Hack to have a yoTime in the state estimator
      
      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
      pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();
      
      if (visualize)
         updateVisualizers();
   }

   private void updateVisualizers()
   {
      for (int i = 0; i < dynamicGraphicMeasurementFrames.size(); i++)
         dynamicGraphicMeasurementFrames.get(i).update();
   }

   public void startIMUDriftEstimation()
   {
      pelvisLinearStateUpdater.startIMUDriftEstimation();
   }

   public void startIMUDriftCompensation()
   {
      pelvisLinearStateUpdater.startIMUDriftCompensation();
   }
   
   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      // Do nothing for the orientation since the IMU is trusted
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

   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolderForEstimator)
   {
      sensorReader.setForceSensorDataHolder(forceSensorDataHolderForEstimator);
   }

   public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedOrientation(estimatedOrientationToPack);
   }

   public void setEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedAngularVelocity(estimatedAngularVelocityToPack);
   }

   public void setEstimatedAngularVelocity(FrameVector estimatedAngularVelocity)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
   {
   }

   public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisPosition(estimatedPelvisPositionToPack);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocityToPack);
   }

   public DenseMatrix64F getCovariance()
   {
      return null;
   }

   public DenseMatrix64F getState()
   {
      return null;
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
   {
   }

   public void initializeOrientationEstimateToMeasurement()
   {
      // Do nothing
   }
}
