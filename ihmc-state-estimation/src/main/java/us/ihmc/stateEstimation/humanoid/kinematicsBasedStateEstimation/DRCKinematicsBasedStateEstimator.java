package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.StateEstimatorModeSubscriber;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.imu.FusedIMUSensor;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.DRCStateEstimatorInterface;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   public static final boolean INITIALIZE_HEIGHT_WITH_FOOT = true;
   public static final boolean USE_NEW_PELVIS_POSE_CORRECTOR = true;
   public static final boolean ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoDouble yoTime = new YoDouble("t_stateEstimator", registry);
   private final AtomicReference<StateEstimatorMode> atomicOperationMode = new AtomicReference<>(null);
   private final YoEnum<StateEstimatorMode> operatingMode = new YoEnum<>("stateEstimatorOperatingMode", registry, StateEstimatorMode.class, false);

   private final FusedIMUSensor fusedIMUSensor;
   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
   private final ForceSensorStateUpdater forceSensorStateUpdater;
   private final IMUBiasStateEstimator imuBiasStateEstimator;
   private final IMUYawDriftEstimator imuYawDriftEstimator;

   private final PelvisPoseHistoryCorrectionInterface pelvisPoseHistoryCorrection;

   private final double estimatorDT;

   private boolean visualizeMeasurementFrames = false;
   private final ArrayList<YoGraphicReferenceFrame> yoGraphicMeasurementFrames = new ArrayList<>();

   private final CenterOfPressureVisualizer copVisualizer;

   private final YoBoolean usePelvisCorrector;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;

   private final JointTorqueFromForceSensorVisualizer jointTorqueFromForceSensorVisualizer;

   private StateEstimatorModeSubscriber stateEstimatorModeSubscriber = null;

   private final YoBoolean reinitializeStateEstimator = new YoBoolean("reinitializeStateEstimator", registry);

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, StateEstimatorParameters stateEstimatorParameters,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, ForceSensorDataHolder forceSensorDataHolderToUpdate,
         CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate, String[] imuSensorsToUseInStateEstimator,
         double gravitationalAcceleration, Map<RigidBody, FootSwitchInterface> footSwitches,
         CenterOfPressureDataHolder centerOfPressureDataHolderFromController, RobotMotionStatusHolder robotMotionStatusFromController,
         Map<RigidBody, ? extends ContactablePlaneBody> feet, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      usePelvisCorrector = new YoBoolean("useExternalPelvisCorrector", registry);
      usePelvisCorrector.set(true);
      if(forceSensorDataHolderToUpdate != null)
      {
         forceSensorStateUpdater = new ForceSensorStateUpdater(sensorOutputMapReadOnly, forceSensorDataHolderToUpdate, stateEstimatorParameters, gravitationalAcceleration, yoGraphicsListRegistry, registry);
      }
      else
      {
         forceSensorStateUpdater = null;
      }

      if(USE_NEW_PELVIS_POSE_CORRECTOR)
         this.pelvisPoseHistoryCorrection = new NewPelvisPoseHistoryCorrection(inverseDynamicsStructure, stateEstimatorParameters.getEstimatorDT(), registry, yoGraphicsListRegistry, 1000);
      else
         this.pelvisPoseHistoryCorrection = new PelvisPoseHistoryCorrection(inverseDynamicsStructure, stateEstimatorParameters.getEstimatorDT(), registry, 1000);

      List<IMUSensorReadOnly> imuProcessedOutputs = new ArrayList<>();
      List<String> imuSensorsToUse = Arrays.asList(imuSensorsToUseInStateEstimator);
      for (IMUSensorReadOnly imu : sensorOutputMapReadOnly.getIMUProcessedOutputs())
      {
         if (imuSensorsToUse.contains(imu.getSensorName()))
            imuProcessedOutputs.add(imu);
      }

      List<IMUSensorReadOnly> imusToUse = new ArrayList<>();

      if (stateEstimatorParameters.createFusedIMUSensor())
      {
         if (imuProcessedOutputs.size() != 2)
            throw new RuntimeException("Cannot create FusedIMUSensor.");
         fusedIMUSensor = new FusedIMUSensor(imuProcessedOutputs.get(0), imuProcessedOutputs.get(1), estimatorDT,
               stateEstimatorParameters.getIMUYawDriftFilterFreqInHertz(), registry);
         imusToUse.add(fusedIMUSensor);
      }
      else
      {
         fusedIMUSensor = null;
         imusToUse.addAll(imuProcessedOutputs);
      }

      BooleanProvider cancelGravityFromAccelerationMeasurement = new BooleanParameter("cancelGravityFromAccelerationMeasurement", registry, stateEstimatorParameters.cancelGravityFromAccelerationMeasurement());
      
      imuBiasStateEstimator = new IMUBiasStateEstimator(imuProcessedOutputs, feet.keySet(), gravitationalAcceleration, cancelGravityFromAccelerationMeasurement,
                                                        estimatorDT, stateEstimatorParameters, registry);
      imuYawDriftEstimator = new IMUYawDriftEstimator(inverseDynamicsStructure, footSwitches, feet, estimatorDT, registry);
      imuYawDriftEstimator.configureModuleParameters(stateEstimatorParameters);

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, stateEstimatorParameters, registry);
      if (imusToUse.size() > 0)
      {
         pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imusToUse, imuBiasStateEstimator, imuYawDriftEstimator, estimatorDT, registry);
      }
      else
      {
         PrintTools.warn(this, "No IMUs, setting up with: " + ConstantPelvisRotationalStateUpdater.class.getSimpleName(), true);
         pelvisRotationalStateUpdater = new ConstantPelvisRotationalStateUpdater(inverseDynamicsStructure, registry);
      }

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, imusToUse, imuBiasStateEstimator, cancelGravityFromAccelerationMeasurement, footSwitches,
            estimatorCenterOfMassDataHolderToUpdate,
            centerOfPressureDataHolderFromController, feet, gravitationalAcceleration, stateEstimatorParameters,
            yoGraphicsListRegistry, registry);


      if (yoGraphicsListRegistry != null)
      {
         copVisualizer = new CenterOfPressureVisualizer(footSwitches, yoGraphicsListRegistry, registry);
      }
      else
      {
         copVisualizer = null;
      }

      if (ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ)
         jointTorqueFromForceSensorVisualizer = new JointTorqueFromForceSensorVisualizer(footSwitches, registry);
      else
         jointTorqueFromForceSensorVisualizer = null;

      visualizeMeasurementFrames = visualizeMeasurementFrames && yoGraphicsListRegistry != null;

      List<IMUSensorReadOnly> imusToDisplay = new ArrayList<>();
      imusToDisplay.addAll(imuProcessedOutputs);
      if (fusedIMUSensor != null)
         imusToDisplay.add(fusedIMUSensor);

      if (visualizeMeasurementFrames)
         setupYoGraphics(yoGraphicsListRegistry, imusToDisplay);

      if (stateEstimatorParameters.requestFrozenModeAtStart())
         operatingMode.set(StateEstimatorMode.FROZEN);
      else
         operatingMode.set(StateEstimatorMode.NORMAL);
   }

   private void setupYoGraphics(YoGraphicsListRegistry yoGraphicsListRegistry, List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         YoGraphicReferenceFrame yoGraphicMeasurementFrame = new YoGraphicReferenceFrame(imuProcessedOutputs.get(i).getMeasurementFrame(), registry, 1.0);
         yoGraphicMeasurementFrames.add(yoGraphicMeasurementFrame);
      }
      yoGraphicsListRegistry.registerYoGraphics("imuFrame", yoGraphicMeasurementFrames);
   }

   @Override
   public StateEstimator getStateEstimator()
   {
      return this;
   }

   @Override
   public void initialize()
   {
      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.initialize();
      if (pelvisRotationalStateUpdater != null)
      {
         if (operatingMode.getEnumValue() == StateEstimatorMode.FROZEN)
            pelvisRotationalStateUpdater.initializeForFrozenState();
         else
            pelvisRotationalStateUpdater.initialize();
      }
      if(forceSensorStateUpdater != null)
      {
         forceSensorStateUpdater.initialize();
      }
      pelvisLinearStateUpdater.initialize();

      imuBiasStateEstimator.initialize();
   }

   @Override
   public void doControl()
   {
      if(reinitializeStateEstimator.getBooleanValue())
      {
         reinitializeStateEstimator.set(false);
         initialize();
      }
      yoTime.set(Conversions.nanosecondsToSeconds(sensorOutputMapReadOnly.getTimestamp()));

      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      if (stateEstimatorModeSubscriber != null && stateEstimatorModeSubscriber.checkForNewOperatingModeRequest())
      {
         operatingMode.set(stateEstimatorModeSubscriber.getRequestedOperatingMode());
      }

      if (atomicOperationMode.get() != null)
         operatingMode.set(atomicOperationMode.getAndSet(null));

      jointStateUpdater.updateJointState();

      switch (operatingMode.getEnumValue())
      {
         case FROZEN:
            if (pelvisRotationalStateUpdater != null)
            {
               pelvisRotationalStateUpdater.updateForFrozenState();
            }
            if(forceSensorStateUpdater != null)
            {
               forceSensorStateUpdater.updateForceSensorState();
            }
            pelvisLinearStateUpdater.updateForFrozenState();
            break;

         case NORMAL:
         default:
            if (pelvisRotationalStateUpdater != null)
            {
               pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
            }

            if(forceSensorStateUpdater != null)
            {
               forceSensorStateUpdater.updateForceSensorState();
            }
            pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();

            List<RigidBody> trustedFeet = pelvisLinearStateUpdater.getCurrentListOfTrustedFeet();
            imuBiasStateEstimator.compute(trustedFeet);
            break;
      }

      if (usePelvisCorrector.getBooleanValue() && pelvisPoseHistoryCorrection != null)
      {
         pelvisPoseHistoryCorrection.doControl(sensorOutputMapReadOnly.getVisionSensorTimestamp());
      }

      updateVisualizers();
   }

   private void updateVisualizers()
   {
      if (copVisualizer != null)
         copVisualizer.update();

      if (visualizeMeasurementFrames)
      {
         for (int i = 0; i < yoGraphicMeasurementFrames.size(); i++)
            yoGraphicMeasurementFrames.get(i).update();
      }

      if (ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ)
         jointTorqueFromForceSensorVisualizer.update();
   }

   @Override
   public void initializeEstimatorToActual(Tuple3DReadOnly initialCoMPosition, QuaternionReadOnly initialEstimationLinkOrientation)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      // Do nothing for the orientation since the IMU is trusted
   }
   
   public void initializeEstimatorToActual(Tuple3DReadOnly initialCoMPosition, Tuple3DReadOnly initialCoMVelocity, QuaternionReadOnly initialOrientation, Tuple3DReadOnly iniitalAngularVelocity)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      //Do nothing for the orientation since the IMU is trusted 
      pelvisLinearStateUpdater.initializeCoMVelocityToActual(initialCoMVelocity);
   }
   
   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void getEstimatedOrientation(FrameQuaternion estimatedOrientationToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedOrientation(estimatedOrientationToPack);
   }

   @Override
   public void setEstimatedOrientation(FrameQuaternion estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   @Override
   public void getEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocityToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedAngularVelocity(estimatedAngularVelocityToPack);
   }

   @Override
   public void setEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocity)
   {
      // Do nothing, IMU is trusted
   }

   @Override
   public void getEstimatedCoMPosition(FramePoint3D estimatedCoMPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   @Override
   public void setEstimatedCoMPosition(FramePoint3D estimatedCoMPosition)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   @Override
   public void getEstimatedCoMVelocity(FrameVector3D estimatedCoMVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   @Override
   public void setEstimatedCoMVelocity(FrameVector3D estimatedCoMVelocity)
   {
   }

   @Override
   public void getEstimatedPelvisPosition(FramePoint3D estimatedPelvisPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisPosition(estimatedPelvisPositionToPack);
   }

   @Override
   public void getEstimatedPelvisLinearVelocity(FrameVector3D estimatedPelvisLinearVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocityToPack);
   }

   @Override
   public DenseMatrix64F getCovariance()
   {
      return null;
   }

   @Override
   public DenseMatrix64F getState()
   {
      return null;
   }

   @Override
   public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
   {
   }

   @Override
   public void initializeOrientationEstimateToMeasurement()
   {
      // Do nothing
   }

   public ForceSensorDataHolderReadOnly getForceSensorOutput()
   {
      return forceSensorStateUpdater.getForceSensorOutput();
   }

   public ForceSensorDataHolderReadOnly getForceSensorOutputWithGravityCancelled()
   {
      return forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled();
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      pelvisPoseHistoryCorrection.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }

   public void setOperatingModeSubscriber(StateEstimatorModeSubscriber stateEstimatorModeSubscriber)
   {
      this.stateEstimatorModeSubscriber = stateEstimatorModeSubscriber;
   }

   public void setRequestWristForceSensorCalibrationSubscriber(RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber)
   {
      forceSensorStateUpdater.setRequestWristForceSensorCalibrationSubscriber(requestWristForceSensorCalibrationSubscriber);
   }

   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return forceSensorStateUpdater;
   }

   public void requestStateEstimatorMode(StateEstimatorMode stateEstimatorMode)
   {
      atomicOperationMode.set(stateEstimatorMode);
   }
}
