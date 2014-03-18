package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.SensorConfigurationFactory;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SensorReader sensorReader;

   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdater pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
 
   private final SensorNoiseParameters sensorNoiseParametersForEstimator = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();
   
   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure,
         RigidBodyToIndexMap estimatorRigidBodyToIndexMap, double estimateDT, SensorReaderFactory sensorReaderFactory, double gravitationalAcceleration,
         SideDependentList<WrenchBasedFootSwitch> footSwitches, SideDependentList<ContactablePlaneBody> bipedFeet,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      sensorReader = sensorReaderFactory.getSensorReader();

      SensorFilterParameters sensorFilterParameters = new SensorFilterParameters(
            DRCConfigParameters.JOINT_POSITION_FILTER_FREQ_HZ, DRCConfigParameters.JOINT_VELOCITY_FILTER_FREQ_HZ, 
            DRCConfigParameters.ORIENTATION_FILTER_FREQ_HZ, DRCConfigParameters.ANGULAR_VELOCITY_FILTER_FREQ_HZ,
            DRCConfigParameters.LINEAR_ACCELERATION_FILTER_FREQ_HZ, estimateDT);

      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = new JointAndIMUSensorDataSource(sensorReaderFactory.getStateEstimatorSensorDefinitions(), sensorFilterParameters, registry);
      JointAndIMUSensorMap jointAndIMUSensorMap = jointAndIMUSensorDataSource.getSensorMap();

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, jointAndIMUSensorDataSource, registry);

      SensorConfigurationFactory sensorConfigurationFactory = new SensorConfigurationFactory(sensorNoiseParametersForEstimator, gravitationalAcceleration);
      
      List<OrientationSensorConfiguration> orientationSensorConfigurations = sensorConfigurationFactory.createOrientationSensorConfigurations(jointAndIMUSensorMap.getOrientationSensors());
      List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = sensorConfigurationFactory.createAngularVelocitySensorConfigurations(jointAndIMUSensorMap.getAngularVelocitySensors());
      
      pelvisRotationalStateUpdater = new PelvisRotationalStateUpdater(orientationSensorConfigurations, angularVelocitySensorConfigurations, inverseDynamicsStructure, registry);

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, footSwitches, bipedFeet,
            gravitationalAcceleration, estimateDT, dynamicGraphicObjectsListRegistry, registry);
//      pelvisLinearStateUpdater.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);

      sensorReader.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);
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
      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updatePelvisOrientationAndAngularVelocity();
      pelvisLinearStateUpdater.updatePelvisPositionAndLinearVelocity();
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

   public void getEstimatedOrientation(FrameOrientation frameOrientationToPack)
   {
      throw new RuntimeException("Should not get there, IMU is trusted");
   }

   public void setEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      throw new RuntimeException("Should not get there, IMU is trusted");
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
