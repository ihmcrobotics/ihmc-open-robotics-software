package us.ihmc.wholeBodyController.diagnostics;

import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.WrenchSensorValidityChecker;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiagnosticControllerToolbox
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble yoTime;

   private final DiagnosticParameters diagnosticParameters;

   private final Map<OneDoFJointBasics, OneDoFJointSensorValidityChecker> jointSensorValidityCheckers;
   private final Map<String, IMUSensorValidityChecker> imuSensorValidityCheckers;
   private final Map<ForceSensorDefinition, WrenchSensorValidityChecker> wrenchSensorValidityCheckers;

   private final Map<OneDoFJointBasics, PositionVelocity1DConsistencyChecker> jointPositionVelocityConsistencyCheckers;
   private final Map<String, OrientationAngularVelocityConsistencyChecker> orientationAngularVelocityConsistencyCheckers;

   private final Map<OneDoFJointBasics, OneDoFJointForceTrackingDelayEstimator> jointForceTrackingDelayEstimators;
   private final Map<OneDoFJointBasics, OneDoFJointFourierAnalysis> jointFourierAnalysisMap;

   private final double dt;
   private final RigidBodyBasics rootBody;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] joints;
   private final Map<String, OneDoFJointBasics> nameToJointMap;
   private final JointDesiredOutputList lowLevelOutput;
   private final SensorOutputMapReadOnly sensorOutputMap;

   public DiagnosticControllerToolbox(RigidBodyBasics rootBody,
                                      FloatingJointBasics rootJoint,
                                      JointDesiredOutputList lowLevelOutput,
                                      SensorOutputMapReadOnly sensorOutputMap,
                                      DiagnosticParameters diagnosticParameters,
                                      YoDouble yoTime,
                                      YoRegistry parentRegistry)
   {
      this.lowLevelOutput = lowLevelOutput;
      this.sensorOutputMap = sensorOutputMap;
      this.diagnosticParameters = diagnosticParameters;
      this.yoTime = yoTime;

      this.rootBody = rootBody;
      this.rootJoint = rootJoint;
      joints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toArray(OneDoFJointBasics[]::new);
      nameToJointMap = Stream.of(joints).collect(Collectors.toMap(JointBasics::getName, Function.identity()));

      DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration = diagnosticParameters.getOrCreateSensorProcessingConfiguration(null,
                                                                                                                                                      null);
      dt = diagnosticSensorProcessingConfiguration.getEstimatorDT();
      jointSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getJointSensorValidityCheckers();
      imuSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getIMUSensorValidityCheckers();
      wrenchSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getWrenchSensorValidityCheckers();

      jointPositionVelocityConsistencyCheckers = diagnosticSensorProcessingConfiguration.getJointPositionVelocityConsistencyCheckers();
      orientationAngularVelocityConsistencyCheckers = diagnosticSensorProcessingConfiguration.getIMUOrientationAngularVelocityConsistencyCheckers();

      jointForceTrackingDelayEstimators = diagnosticSensorProcessingConfiguration.getJointForceTrackingDelayEstimators();
      jointFourierAnalysisMap = diagnosticSensorProcessingConfiguration.getJointFourierAnalysisMap();

      parentRegistry.addChild(registry);
   }

   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getJoints()
   {
      return joints;
   }

   public OneDoFJointBasics getJoint(String jointName)
   {
      return nameToJointMap.get(jointName);
   }

   public List<OneDoFJointBasics> getJoints(String... jointNames)
   {
      return Stream.of(jointNames).map(this::getJoint).collect(Collectors.toList());
   }

   public JointDesiredOutputList getLowLevelOutput()
   {
      return lowLevelOutput;
   }

   public DiagnosticParameters getDiagnosticParameters()
   {
      return diagnosticParameters;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public double getDT()
   {
      return dt;
   }

   public OneDoFJointSensorValidityChecker getJointSensorValidityChecker(OneDoFJointBasics joint)
   {
      return jointSensorValidityCheckers.get(joint);
   }

   public IMUSensorValidityChecker getIMUSensorValidityChecker(String imuName)
   {
      return imuSensorValidityCheckers.get(imuName);
   }

   public WrenchSensorValidityChecker getWrenchSensorValidityChecker(ForceSensorDefinition wrenchSensorDefinition)
   {
      return wrenchSensorValidityCheckers.get(wrenchSensorDefinition);
   }

   public PositionVelocity1DConsistencyChecker getJointPositionVelocityConsistencyChecker(OneDoFJointBasics joint)
   {
      return jointPositionVelocityConsistencyCheckers.get(joint);
   }

   public OrientationAngularVelocityConsistencyChecker getIMUOrientationAngularVelocityConsistencyChecker(String imuName)
   {
      return orientationAngularVelocityConsistencyCheckers.get(imuName);
   }

   public OneDoFJointForceTrackingDelayEstimator getJointForceTrackingDelayEstimator(OneDoFJointBasics joint)
   {
      return jointForceTrackingDelayEstimators.get(joint);
   }

   public OneDoFJointFourierAnalysis getJointFourierAnalysis(OneDoFJointBasics joint)
   {
      return jointFourierAnalysisMap.get(joint);
   }

   public IMUSensorReadOnly getIMUSensorReadOnly(String imuName)
   {
      List<? extends IMUSensorReadOnly> imuProcessedOutputs = sensorOutputMap.getIMUOutputs();
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         if (imuProcessedOutputs.get(i).getSensorName().equals(imuName))
            return imuProcessedOutputs.get(i);
      }
      return null;
   }
}
