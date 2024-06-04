package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.wrapFootSwitchInterfaces;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
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
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.DistributedIMUBasedCenterOfMassStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.MomentumStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.SimpleMomentumStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class DRCKinematicsBasedStateEstimator implements StateEstimatorController
{
   public static final boolean USE_NEW_PELVIS_POSE_CORRECTOR = true;
   private static final boolean ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ = true;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoDouble yoTime = new YoDouble("t_stateEstimator", registry);
   private final AtomicReference<StateEstimatorMode> atomicOperationMode = new AtomicReference<>(null);
   private final YoEnum<StateEstimatorMode> operatingMode = new YoEnum<>("stateEstimatorOperatingMode", registry, StateEstimatorMode.class, false);

   private final JointStateUpdater jointStateUpdater;
   private final ForceSensorStateUpdater forceSensorStateUpdater;
   private final PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;
   private final MomentumStateUpdater momentumStateUpdater;
   private final IMUBiasStateEstimator imuBiasStateEstimator;
   private final IMUYawDriftEstimator imuYawDriftEstimator;

   private final PelvisPoseHistoryCorrectionInterface pelvisPoseHistoryCorrection;

   private final double estimatorDT;

   private boolean visualizeMeasurementFrames = false;
   private final List<IMUSensorReadOnly> imusToVisualize;
   private final List<YoFramePose3D> measurementFramePoses = new ArrayList<>();

   private final CenterOfPressureVisualizer copVisualizer;

   private final YoBoolean usePelvisCorrector;
   private final SensorOutputMapReadOnly sensorOutput;

   private final JointTorqueAgainstForceSensorVisualizer jointTorqueAgainstForceSensorVisualizer;

   private final List<FootSwitchInterface> footSwitchList;

   private final YoBoolean reinitializeStateEstimator = new YoBoolean("reinitializeStateEstimator", registry);

   private final FloatingJointBasics rootJoint;
   private final YoFixedFrameTwist yoRootTwist;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure,
                                           StateEstimatorParameters stateEstimatorParameters,
                                           SensorOutputMapReadOnly sensorOutputMap,
                                           CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate,
                                           String[] imuSensorsToUseInStateEstimator,
                                           double gravitationalAcceleration,
                                           Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                           CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                           RobotMotionStatusHolder robotMotionStatusFromController,
                                           Map<RigidBodyBasics, ? extends ContactablePlaneBody> feet,
                                           ForceSensorDataHolder forceSensorDataHolderToUpdate,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.sensorOutput = sensorOutputMap;
      this.footSwitchList = new ArrayList<>(footSwitches.values());

      rootJoint = inverseDynamicsStructure.getRootJoint();

      usePelvisCorrector = new YoBoolean("useExternalPelvisCorrector", registry);
      usePelvisCorrector.set(true);

      if (USE_NEW_PELVIS_POSE_CORRECTOR)
      {
         ClippedSpeedOffsetErrorInterpolatorParameters parameters = new ClippedSpeedOffsetErrorInterpolatorParameters();
         parameters.setIsRotationCorrectionEnabled(true);
         parameters.setBreakFrequency(10.0);
         parameters.setDeadZoneSizes(0.0, 0.0, 0.0, 0.0);
         parameters.setMaxTranslationalCorrectionSpeed(0.5);
         parameters.setMaxRotationalCorrectionSpeed(0.5);

         this.pelvisPoseHistoryCorrection = new NewPelvisPoseHistoryCorrection(inverseDynamicsStructure,
                                                                               stateEstimatorParameters.getEstimatorDT(),
                                                                               registry,
                                                                               yoGraphicsListRegistry,
                                                                               1000,
                                                                               parameters);
      }
      else
      {
         this.pelvisPoseHistoryCorrection = new PelvisPoseHistoryCorrection(inverseDynamicsStructure,
                                                                            stateEstimatorParameters.getEstimatorDT(),
                                                                            registry,
                                                                            1000);
      }

      List<IMUSensorReadOnly> imuProcessedOutputs = new ArrayList<>();
      List<String> imuSensorsToUse = Arrays.asList(imuSensorsToUseInStateEstimator);
      for (IMUSensorReadOnly imu : sensorOutputMap.getIMUOutputs())
      {
         if (imuSensorsToUse.contains(imu.getSensorName()))
            imuProcessedOutputs.add(imu);
      }

      List<IMUSensorReadOnly> imusToUse = new ArrayList<>(imuProcessedOutputs);

      BooleanProvider cancelGravityFromAccelerationMeasurement = new BooleanParameter("cancelGravityFromAccelerationMeasurement",
                                                                                      registry,
                                                                                      stateEstimatorParameters.cancelGravityFromAccelerationMeasurement());

      imuBiasStateEstimator = new IMUBiasStateEstimator(imuProcessedOutputs,
                                                        feet.keySet(),
                                                        gravitationalAcceleration,
                                                        cancelGravityFromAccelerationMeasurement,
                                                        estimatorDT,
                                                        stateEstimatorParameters,
                                                        registry);
      imuYawDriftEstimator = new IMUYawDriftEstimator(inverseDynamicsStructure,
                                                      footSwitches,
                                                      feet,
                                                      robotMotionStatusFromController,
                                                      stateEstimatorParameters,
                                                      registry);

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMap, stateEstimatorParameters, registry);

      if (forceSensorDataHolderToUpdate != null)
      {
         forceSensorStateUpdater = new ForceSensorStateUpdater(rootJoint,
                                                               sensorOutputMap,
                                                               forceSensorDataHolderToUpdate,
                                                               stateEstimatorParameters,
                                                               gravitationalAcceleration,
                                                               robotMotionStatusFromController,
                                                               yoGraphicsListRegistry,
                                                               registry);
      }
      else
      {
         forceSensorStateUpdater = null;
      }

      if (imusToUse.size() > 0)
      {
         pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure,
                                                                                 imusToUse,
                                                                                 imuBiasStateEstimator,
                                                                                 imuYawDriftEstimator,
                                                                                 estimatorDT,
                                                                                 registry);
      }
      else
      {
         LogTools.warn("No IMUs, setting up with: " + ConstantPelvisRotationalStateUpdater.class.getSimpleName(), true);
         pelvisRotationalStateUpdater = new ConstantPelvisRotationalStateUpdater(inverseDynamicsStructure, registry);
      }

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure,
                                                              imusToUse,
                                                              imuBiasStateEstimator,
                                                              cancelGravityFromAccelerationMeasurement,
                                                              footSwitches,
                                                              centerOfPressureDataHolderFromController,
                                                              feet,
                                                              gravitationalAcceleration,
                                                              stateEstimatorParameters,
                                                              yoGraphicsListRegistry,
                                                              registry);

      switch (stateEstimatorParameters.getMomentumEstimatorMode())
      {
         case DISTRIBUTED_IMUS:
            momentumStateUpdater = new DistributedIMUBasedCenterOfMassStateUpdater(rootJoint,
                                                                                   sensorOutputMap.getIMUOutputs(),
                                                                                   pelvisLinearStateUpdater.getCurrentListOfTrustedFeet(),
                                                                                   estimatorDT,
                                                                                   gravitationalAcceleration,
                                                                                   estimatorCenterOfMassDataHolderToUpdate);
            break;
         case SIMPLE:
            momentumStateUpdater = new SimpleMomentumStateUpdater(rootJoint,
                                                                  gravitationalAcceleration,
                                                                  stateEstimatorParameters,
                                                                  footSwitches,
                                                                  estimatorCenterOfMassDataHolderToUpdate,
                                                                  yoGraphicsListRegistry);
            break;
         case WRENCH_BASED:
            momentumStateUpdater = new WrenchBasedMomentumStateUpdater(rootJoint,
                                                                       wrapFootSwitchInterfaces(footSwitchList),
                                                                       estimatorDT,
                                                                       gravitationalAcceleration,
                                                                       estimatorCenterOfMassDataHolderToUpdate);
            break;
         case NONE:
            momentumStateUpdater = null;
            break;
         default:
            throw new IllegalArgumentException("Unhandled mode: " + stateEstimatorParameters.getMomentumEstimatorMode());
      }

      if (momentumStateUpdater != null)
         registry.addChild(momentumStateUpdater.getRegistry());

      if (yoGraphicsListRegistry != null)
      {
         copVisualizer = new CenterOfPressureVisualizer(footSwitches, yoGraphicsListRegistry, registry);
      }
      else
      {
         copVisualizer = null;
      }

      if (ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ)
         jointTorqueAgainstForceSensorVisualizer = new JointTorqueAgainstForceSensorVisualizer(rootJoint.getSuccessor(), feet, footSwitches, registry);
      else
         jointTorqueAgainstForceSensorVisualizer = null;

      visualizeMeasurementFrames = visualizeMeasurementFrames && yoGraphicsListRegistry != null;

      imusToVisualize = new ArrayList<>();
      imusToVisualize.addAll(imuProcessedOutputs);

      if (visualizeMeasurementFrames)
         setupYoGraphics(yoGraphicsListRegistry, imusToVisualize);

      if (stateEstimatorParameters.requestFrozenModeAtStart())
         operatingMode.set(StateEstimatorMode.FROZEN);
      else
         operatingMode.set(StateEstimatorMode.NORMAL);

      yoRootTwist = new YoFixedFrameTwist("RootTwist",
                                          rootJoint.getFrameAfterJoint(),
                                          rootJoint.getFrameBeforeJoint(),
                                          rootJoint.getFrameAfterJoint(),
                                          registry);
   }

   private void setupYoGraphics(YoGraphicsListRegistry yoGraphicsListRegistry, List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         measurementFramePoses.add(new YoFramePose3D(imuProcessedOutputs.get(i).getSensorName() + "Frame", ReferenceFrame.getWorldFrame(), registry));
         yoGraphicsListRegistry.registerYoGraphic("imuFrame",
                                                  new YoGraphicCoordinateSystem(imuProcessedOutputs.get(i).getSensorName() + "Frame",
                                                                                measurementFramePoses.get(i),
                                                                                1.0));
      }
   }

   @Override
   public void initialize()
   {
      jointStateUpdater.initialize();
      if (pelvisRotationalStateUpdater != null)
      {
         pelvisRotationalStateUpdater.initialize();
      }

      if (forceSensorStateUpdater != null)
         forceSensorStateUpdater.initialize();

      pelvisLinearStateUpdater.initialize();

      if (momentumStateUpdater != null)
         momentumStateUpdater.initialize();

      imuBiasStateEstimator.initialize();
      imuYawDriftEstimator.initialize();
   }

   @Override
   public void doControl()
   {
      if (reinitializeStateEstimator.getBooleanValue())
      {
         reinitializeStateEstimator.set(false);
         initialize();
      }
      yoTime.set(Conversions.nanosecondsToSeconds(sensorOutput.getWallTime()));

      if (atomicOperationMode.get() != null)
      {
         operatingMode.set(atomicOperationMode.getAndSet(null));
         LogTools.debug("Estimator went to {}", operatingMode.getEnumValue());
      }

      jointStateUpdater.updateJointState();

      if (pelvisRotationalStateUpdater != null)
         pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();

      if (forceSensorStateUpdater != null)
         forceSensorStateUpdater.updateForceSensorState();

      for (int i = 0; i < footSwitchList.size(); i++)
         footSwitchList.get(i).update();

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

      if (momentumStateUpdater != null)
         momentumStateUpdater.update();

      yoRootTwist.setMatchingFrame(rootJoint.getJointTwist());

      List<RigidBodyBasics> trustedFeet = pelvisLinearStateUpdater.getCurrentListOfTrustedFeet();
      imuBiasStateEstimator.compute(trustedFeet);

      if (usePelvisCorrector.getBooleanValue() && pelvisPoseHistoryCorrection != null)
      {
         pelvisPoseHistoryCorrection.doControl(sensorOutput.getWallTime());
      }

      updateVisualizers();
   }

   private void updateVisualizers()
   {
      if (copVisualizer != null)
         copVisualizer.update();

      if (visualizeMeasurementFrames)
      {
         for (int i = 0; i < measurementFramePoses.size(); i++)
            measurementFramePoses.get(i).setFromReferenceFrame(imusToVisualize.get(i).getMeasurementFrame());
      }

      if (ENABLE_JOINT_TORQUES_FROM_FORCE_SENSORS_VIZ)
         jointTorqueAgainstForceSensorVisualizer.update();
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      pelvisLinearStateUpdater.initializeRootJointPosition(rootJointTransform.getTranslation());
      reinitializeStateEstimator.set(true);
      // Do nothing for the orientation since the IMU is trusted
   }

   @Override
   public YoRegistry getYoRegistry()
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

   public void requestReinitializeEstimator()
   {
      reinitializeStateEstimator.set(true);
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode stateEstimatorMode)
   {
      atomicOperationMode.set(stateEstimatorMode);
   }

   public ForceSensorStateUpdater getForceSensorStateUpdater()
   {
      return forceSensorStateUpdater;
   }

   @Override
   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return forceSensorStateUpdater;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutputWithGravityCancelled()
   {
      return forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(forceSensorStateUpdater.getSCS2YoGraphics());
      group.addChild(pelvisLinearStateUpdater.getSCS2YoGraphics());
      if (momentumStateUpdater != null)
         group.addChild(momentumStateUpdater.getSCS2YoGraphics());
      if (copVisualizer != null)
         group.addChild(copVisualizer.getSCS2YoGraphics());
      for (int i = 0; i < measurementFramePoses.size(); i++)
      {
         YoFramePose3D measurementPose = measurementFramePoses.get(i);
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(imusToVisualize.get(i).getSensorName() + "Frame", measurementPose, 1.0));
      }

      return group;
   }
}
