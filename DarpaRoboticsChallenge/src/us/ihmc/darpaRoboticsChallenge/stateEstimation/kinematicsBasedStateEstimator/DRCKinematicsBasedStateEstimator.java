package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

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
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SensorReader sensorReader;

   private final PelvisStateCalculator pelvisStateCalculator;
   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdater pelvisRotationalStateUpdater;
 
   private final SensorNoiseParameters sensorNoiseParametersForEstimator = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();
   
   private final FullInverseDynamicsStructure inverseDynamicsStructure;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure,
         RigidBodyToIndexMap estimatorRigidBodyToIndexMap, double estimateDT, SensorReaderFactory sensorReaderFactory, double gravitationalAcceleration,
         SideDependentList<WrenchBasedFootSwitch> footSwitches, SideDependentList<ContactablePlaneBody> bipedFeet,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      sensorReader = sensorReaderFactory.getSensorReader();
      this.inverseDynamicsStructure = inverseDynamicsStructure;

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

      pelvisStateCalculator = new PelvisStateCalculator(inverseDynamicsStructure, footSwitches, bipedFeet,
            gravitationalAcceleration, estimateDT, dynamicGraphicObjectsListRegistry, registry);
      pelvisStateCalculator.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);

      sensorReader.setJointAndIMUSensorDataSource(jointAndIMUSensorDataSource);

      yoEstimatedPelvisPosition = new YoFramePoint("estimatedPelvisPosition", worldFrame, registry);
      yoEstimatedPelvisLinearVelocity = new YoFrameVector("estimatedPelvisLinearVelocity", worldFrame, registry);
   }

   public StateEstimator getStateEstimator()
   {
      return this;
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      pelvisRotationalStateUpdater.initialize();
      pelvisStateCalculator.initialize();
      updateRootState();
   }
   
   public void doControl()
   {
      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updatePelvisOrientationAndAngularVelocity();
      pelvisStateCalculator.run();
      updateRootState();
   }

   private final YoFramePoint yoEstimatedPelvisPosition;
   private final YoFrameVector yoEstimatedPelvisLinearVelocity;
   private final FramePoint estimatedPelvisPosition = new FramePoint(worldFrame);
   private final FrameVector estimatedPelvisLinearVelocity = new FrameVector(worldFrame);
   private final Vector3d tempPelvisTranslation = new Vector3d();
   private final Vector3d tempPelvisLinearVelocity = new Vector3d();
   

   private void updateRootState()
   {
      pelvisStateCalculator.getEstimatedPelvisPosition(estimatedPelvisPosition);
      pelvisStateCalculator.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocity);
      
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      estimatedPelvisPosition.getPoint(tempPelvisTranslation);
      estimatedPelvisLinearVelocity.getVector(tempPelvisLinearVelocity);

      yoEstimatedPelvisPosition.set(tempPelvisTranslation);
      yoEstimatedPelvisLinearVelocity.set(tempPelvisLinearVelocity);
      
      rootJoint.setPosition(tempPelvisTranslation);
      rootJoint.setLinearVelocityInWorld(tempPelvisLinearVelocity);
   }

   public void startIMUDriftEstimation()
   {
      pelvisStateCalculator.startIMUDriftEstimation();
   }

   public void startIMUDriftCompensation()
   {
      pelvisStateCalculator.startIMUDriftCompensation();
   }
   
   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      // Setting the initial CoM Position here.
      FramePoint estimatedCoMPosition = new FramePoint();
      pelvisStateCalculator.getEstimatedCoMPosition(estimatedCoMPosition);
      estimatedCoMPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      estimatedCoMPosition.set(initialCoMPosition);

      pelvisStateCalculator.initializeCoMPositionToActual(estimatedCoMPosition);
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
      pelvisStateCalculator.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
   {
      pelvisStateCalculator.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
   {
      pelvisStateCalculator.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
   {
   }

   public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
   {
      estimatedPelvisPositionToPack.setAndChangeFrame(estimatedPelvisPosition);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
   {
      estimatedPelvisLinearVelocityToPack.setAndChangeFrame(estimatedPelvisLinearVelocity);
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
