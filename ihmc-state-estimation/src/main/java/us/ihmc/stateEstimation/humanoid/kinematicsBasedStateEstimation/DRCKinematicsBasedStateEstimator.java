package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commonWalkingControlModules.visualizer.EstimatedFromTorquesWrenchVisualizer;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.imu.FusedIMUSensor;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class DRCKinematicsBasedStateEstimator implements StateEstimatorController
{
   public static final boolean INITIALIZE_HEIGHT_WITH_FOOT = true;
   public static final boolean USE_NEW_PELVIS_POSE_CORRECTOR = true;
   public static final boolean ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ = false;
   private static final boolean ENABLE_ESTIMATED_WRENCH_VISUALIZER = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoDouble yoTime = new YoDouble("t_stateEstimator", registry);
   private final AtomicReference<StateEstimatorMode> atomicOperationMode = new AtomicReference<>(null);
   private final YoEnum<StateEstimatorMode> operatingMode = new YoEnum<>("stateEstimatorOperatingMode", registry, StateEstimatorMode.class, false);

   private final FusedIMUSensor fusedIMUSensor;
   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
   private final IMUBiasStateEstimator imuBiasStateEstimator;
   private final IMUYawDriftEstimator imuYawDriftEstimator;

   private final EstimatedFromTorquesWrenchVisualizer estimatedWrenchVisualizer;

   private final PelvisPoseHistoryCorrectionInterface pelvisPoseHistoryCorrection;

   private final double estimatorDT;

   private boolean visualizeMeasurementFrames = false;
   private final ArrayList<YoGraphicReferenceFrame> yoGraphicMeasurementFrames = new ArrayList<>();

   private final CenterOfPressureVisualizer copVisualizer;

   private final YoBoolean usePelvisCorrector;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;

   private final JointTorqueFromForceSensorVisualizer jointTorqueFromForceSensorVisualizer;

   private final List<FootSwitchInterface> footSwitchList;

   private final YoBoolean reinitializeStateEstimator = new YoBoolean("reinitializeStateEstimator", registry);

   private final FloatingJointBasics rootJoint;
   private final YoFixedFrameTwist yoRootTwist;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, StateEstimatorParameters stateEstimatorParameters,
                                           SensorOutputMapReadOnly sensorOutputMapReadOnly, CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate,
                                           String[] imuSensorsToUseInStateEstimator, double gravitationalAcceleration,
                                           Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                           CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                           RobotMotionStatusHolder robotMotionStatusFromController, Map<RigidBodyBasics, ? extends ContactablePlaneBody> feet,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.footSwitchList = new ArrayList<>(footSwitches.values());

      usePelvisCorrector = new YoBoolean("useExternalPelvisCorrector", registry);
      usePelvisCorrector.set(true);

      if (USE_NEW_PELVIS_POSE_CORRECTOR)
         this.pelvisPoseHistoryCorrection = new NewPelvisPoseHistoryCorrection(inverseDynamicsStructure, stateEstimatorParameters.getEstimatorDT(), registry,
                                                                               yoGraphicsListRegistry, 1000);
      else
         this.pelvisPoseHistoryCorrection = new PelvisPoseHistoryCorrection(inverseDynamicsStructure, stateEstimatorParameters.getEstimatorDT(), registry,
                                                                            1000);

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
         fusedIMUSensor = new FusedIMUSensor(imuProcessedOutputs.get(0), imuProcessedOutputs.get(1), estimatorDT, registry);
         imusToUse.add(fusedIMUSensor);
      }
      else
      {
         fusedIMUSensor = null;
         imusToUse.addAll(imuProcessedOutputs);
      }

      BooleanProvider cancelGravityFromAccelerationMeasurement = new BooleanParameter("cancelGravityFromAccelerationMeasurement", registry,
                                                                                      stateEstimatorParameters.cancelGravityFromAccelerationMeasurement());

      imuBiasStateEstimator = new IMUBiasStateEstimator(imuProcessedOutputs, feet.keySet(), gravitationalAcceleration, cancelGravityFromAccelerationMeasurement,
                                                        estimatorDT, stateEstimatorParameters, registry);
      imuYawDriftEstimator = new IMUYawDriftEstimator(inverseDynamicsStructure, footSwitches, feet, robotMotionStatusFromController, stateEstimatorParameters,
                                                      registry);

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, stateEstimatorParameters, registry);
      if (imusToUse.size() > 0)
      {
         pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imusToUse, imuBiasStateEstimator,
                                                                                 imuYawDriftEstimator, estimatorDT, registry);
      }
      else
      {
         LogTools.warn("No IMUs, setting up with: " + ConstantPelvisRotationalStateUpdater.class.getSimpleName(), true);
         pelvisRotationalStateUpdater = new ConstantPelvisRotationalStateUpdater(inverseDynamicsStructure, registry);
      }

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, imusToUse, imuBiasStateEstimator,
                                                              cancelGravityFromAccelerationMeasurement, footSwitches, estimatorCenterOfMassDataHolderToUpdate,
                                                              centerOfPressureDataHolderFromController, feet, gravitationalAcceleration,
                                                              stateEstimatorParameters, yoGraphicsListRegistry, registry);

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

      if (ENABLE_ESTIMATED_WRENCH_VISUALIZER)
      {
         List<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
         for (RigidBodyBasics rigidBody : feet.keySet())
         {
            ContactablePlaneBody contactableBody = feet.get(rigidBody);
            contactablePlaneBodies.add(contactableBody);
         }
         estimatedWrenchVisualizer = EstimatedFromTorquesWrenchVisualizer
               .createWrenchVisualizerWithContactableBodies("EstimatedExternalWrenches", inverseDynamicsStructure.getRootJoint().getSuccessor(),
                                                            contactablePlaneBodies, 1.0, yoGraphicsListRegistry, registry);
      }
      else
      {
         estimatedWrenchVisualizer = null;
      }

      rootJoint = inverseDynamicsStructure.getRootJoint();
      yoRootTwist = new YoFixedFrameTwist("RootTwist", rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                                          registry);
   }

   private void setupYoGraphics(YoGraphicsListRegistry yoGraphicsListRegistry, List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         YoGraphicReferenceFrame yoGraphicMeasurementFrame = new YoGraphicReferenceFrame(imuProcessedOutputs.get(i).getMeasurementFrame(), registry, false, 1.0);
         yoGraphicMeasurementFrames.add(yoGraphicMeasurementFrame);
      }
      yoGraphicsListRegistry.registerYoGraphics("imuFrame", yoGraphicMeasurementFrames);
   }

   @Override
   public void initialize()
   {
      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.initialize();
      if (pelvisRotationalStateUpdater != null)
      {
         pelvisRotationalStateUpdater.initialize();
      }
      pelvisLinearStateUpdater.initialize();

      imuBiasStateEstimator.initialize();
   }

   @Override
   public void doControl()
   {
      if (reinitializeStateEstimator.getBooleanValue())
      {
         reinitializeStateEstimator.set(false);
         initialize();
      }
      yoTime.set(Conversions.nanosecondsToSeconds(sensorOutputMapReadOnly.getTimestamp()));

      for (int i = 0; i < footSwitchList.size(); i++)
         footSwitchList.get(i).updateMeasurement();

      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      if (atomicOperationMode.get() != null)
      {
         operatingMode.set(atomicOperationMode.getAndSet(null));
         LogTools.debug("Estimator went to {}", operatingMode.getEnumValue());
      }

      jointStateUpdater.updateJointState();
      if (pelvisRotationalStateUpdater != null)
      {
         pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
      }

      switch (operatingMode.getEnumValue())
      {
      case FROZEN:
         pelvisLinearStateUpdater.updateForFrozenState();
         break;
      case NORMAL:
      default:
         pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();
         break;
      }

      yoRootTwist.setMatchingFrame(rootJoint.getJointTwist());

      List<RigidBodyBasics> trustedFeet = pelvisLinearStateUpdater.getCurrentListOfTrustedFeet();
      imuBiasStateEstimator.compute(trustedFeet);

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

      if (ENABLE_ESTIMATED_WRENCH_VISUALIZER)
         estimatedWrenchVisualizer.update();
   }

   @Override
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      pelvisLinearStateUpdater.initializeRootJointPosition(rootJointTransform.getTranslationVector());
      reinitializeStateEstimator.set(true);
      // Do nothing for the orientation since the IMU is trusted
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

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      pelvisPoseHistoryCorrection.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode stateEstimatorMode)
   {
      atomicOperationMode.set(stateEstimatorMode);
   }
}
