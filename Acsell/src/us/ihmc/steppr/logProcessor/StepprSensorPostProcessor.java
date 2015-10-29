package us.ihmc.steppr.logProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorFunction;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorHelper;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataRawSensorMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.steppr.hardware.StepprJoint;

public class StepprSensorPostProcessor implements LogDataProcessorFunction
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final LogDataRawSensorMap rawSensorMap;
   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private final SensorProcessing postProcessedSensors;
   private final DRCKinematicsBasedStateEstimator stateEstimator;
   private final LogDataProcessorHelper logDataProcessorHelper;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> estimatedJointPositionMap = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> estimatedJointVelocityMap = new LinkedHashMap<>();

   private final YoFramePoint estimatedRootJointPosition = new YoFramePoint("log_estimatedRootJoint", worldFrame, registry);
   private final YoFrameQuaternion estimatedRootJointQuaternion = new YoFrameQuaternion("log_estimatedRootJoint", worldFrame, registry);
   private final YoFrameOrientation estimatedRootJointOrientation = new YoFrameOrientation("log_estimatedRootJoint", worldFrame, registry);
   private final YoFrameVector estimatedRootJointLinearVelocity = new YoFrameVector("log_estimatedRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector estimatedRootJointAngularVelocity = new YoFrameVector("log_estimatedRootJointAngularVelocity", worldFrame, registry);

   private final ArrayList<YoFrameQuaternion> processedIMUOrientationMap = new ArrayList<>();
   private final ArrayList<YoFrameVector> processedIMUAngularVelocityMap = new ArrayList<>();
   private final ArrayList<YoFrameVector> processedIMULinearAccelerationMap = new ArrayList<>();

   public StepprSensorPostProcessor(final DRCRobotModel robotModel, LogDataProcessorHelper logDataProcessorHelper)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      SensorProcessingConfiguration sensorProcessingConfiguration = new SensorProcessingConfiguration()
      {

         @Override
         public SensorNoiseParameters getSensorNoiseParameters()
         {
            return null;
         }

         @Override
         public double getEstimatorDT()
         {
            return robotModel.getEstimatorDT();
         }

         @Override
         public void configureSensorProcessing(SensorProcessing sensorProcessing)
         {
            double defaultJointStiffness = Double.POSITIVE_INFINITY;
            Map<String, Double> jointSpecificStiffness = new HashMap<>();

            jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
            jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);

            jointSpecificStiffness.put(StepprJoint.LEFT_HIP_X.getSdfName(), 6000.0);
            jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_X.getSdfName(), 6000.0);

            jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Y.getSdfName(), 10000.0);
            jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Y.getSdfName(), 10000.0);

            jointSpecificStiffness.put(StepprJoint.LEFT_KNEE_Y.getSdfName(), 6000.0);
            jointSpecificStiffness.put(StepprJoint.RIGHT_KNEE_Y.getSdfName(), 6000.0);

            Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("_log_stiffness", defaultJointStiffness, jointSpecificStiffness);
            DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("log_jointVelocityAlphaFilter", 16.0);

            DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("log_angularVelocityAlphaFilter", 16.0);
            DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("log_linearAccelerationAlphaFilter", 67.0);

            sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, false);

            sensorProcessing.computeJointVelocityFromFiniteDifference(jointVelocityAlphaFilter, true); //vizonly
            sensorProcessing.addJointVelocityAlphaFilter(jointVelocityAlphaFilter, false);
            sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

            sensorProcessing.addIMUAngularVelocityAlphaFilter(angularVelocityAlphaFilter, false);
            sensorProcessing.addIMULinearAccelerationAlphaFilter(linearAccelerationAlphaFilter, false);
         }
      };

      rawSensorMap = logDataProcessorHelper.getRawSensorMap();
      estimatorFullRobotModel = robotModel.createFullRobotModel();

      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      stateEstimatorSensorDefinitions.addIMUSensorDefinition(estimatorFullRobotModel.getIMUDefinitions());
      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(estimatorFullRobotModel.getRootJoint().getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
            stateEstimatorSensorDefinitions.addJointSensorDefinition((OneDoFJoint) joint);
      }

      
      postProcessedSensors = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(estimatorFullRobotModel.getElevator(),
            estimatorFullRobotModel.getPelvis(), estimatorFullRobotModel.getRootJoint());

      stateEstimator = createStateEstimator(inverseDynamicsStructure, robotModel, logDataProcessorHelper, postProcessedSensors);

      for (OneDoFJoint joint : stateEstimatorSensorDefinitions.getJointSensorDefinitions())
      {
         DoubleYoVariable estimatedJointPosition = new DoubleYoVariable("log_q_" + joint.getName(), registry);
         estimatedJointPositionMap.put(joint, estimatedJointPosition);

         DoubleYoVariable estimatedJointVelocity = new DoubleYoVariable("log_qd_" + joint.getName(), registry);
         estimatedJointVelocityMap.put(joint, estimatedJointVelocity);
      }

      for (IMUDefinition imuDefinition : stateEstimatorSensorDefinitions.getIMUSensorDefinitions())
      {
         String imuName = imuDefinition.getName();
         YoFrameQuaternion processedIMUOrientation = new YoFrameQuaternion("log_q_", imuName, null, registry);
         processedIMUOrientationMap.add(processedIMUOrientation);

         YoFrameVector processedIMUAngularVelocity = new YoFrameVector("log_qd_w", imuName, null, registry);
         processedIMUAngularVelocityMap.add(processedIMUAngularVelocity);
         
         YoFrameVector processedIMULinearAcceleration = new YoFrameVector("log_qdd_", imuName, null, registry);
         processedIMULinearAccelerationMap.add(processedIMULinearAcceleration);
      }

      registry.addChild(stateEstimator.getYoVariableRegistry());
   }

   private DRCKinematicsBasedStateEstimator createStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, DRCRobotModel robotModel,
         final LogDataProcessorHelper logDataProcessorHelper, SensorOutputMapReadOnly postProcessedSensors)
   {
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      String[] imuSensorsToUseInStateEstimator = robotModel.getSensorInformation().getIMUSensorsToUseInStateEstimator();
      double gravitationalAcceleration = -9.80;
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();
      CommonHumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(estimatorFullRobotModel);
      SideDependentList<ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(estimatorFullRobotModel, referenceFrames);
      SideDependentList<FootSwitchInterface> footSwitches = createStateEstimatorFootSwitches(logDataProcessorHelper, bipedFeet);
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // no viz for now

      CenterOfPressureDataHolder centerOfPressureDataHolder = new CenterOfPressureDataHolder(referenceFrames.getSoleFrames())
      {
         @Override
         public void getCenterOfPressure(FramePoint2d centerOfPressureToPack, RobotSide robotSide)
         {
            logDataProcessorHelper.getDesiredCoP(robotSide, centerOfPressureToPack);
         }
      };
      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            postProcessedSensors, null, imuSensorsToUseInStateEstimator, gravitationalAcceleration, footSwitches, centerOfPressureDataHolder, new RobotMotionStatusHolder(), bipedFeet, yoGraphicsListRegistry);
      return stateEstimator;
   }

   private SideDependentList<FootSwitchInterface> createStateEstimatorFootSwitches(final LogDataProcessorHelper logDataProcessorHelper, final SideDependentList<ContactablePlaneBody> bipedFeet)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();
      
      for (final RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";
         String nameSpaceEnding = namePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         YoVariableHolder yoVariableHolder = logDataProcessorHelper.getLogYoVariableHolder();
         final BooleanYoVariable hasFootHitGround = (BooleanYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix + "FilteredFootHitGround");
         final BooleanYoVariable forceMagnitudePastThreshhold = (BooleanYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix +  "ForcePastThresh");
         final DoubleYoVariable footLoadPercentage = (DoubleYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix + "FootLoadPercentage");

         FootSwitchInterface footSwitch = new FootSwitchInterface()
         {
            
            @Override
            public void reset()
            {
            }
            
            @Override
            public boolean hasFootHitGround()
            {
               return hasFootHitGround.getBooleanValue();
            }
            
            @Override
            public ReferenceFrame getMeasurementFrame()
            {
               return null;
            }
            
            @Override
            public boolean getForceMagnitudePastThreshhold()
            {
               return forceMagnitudePastThreshhold.getBooleanValue();
            }
            
            @Override
            public double computeFootLoadPercentage()
            {
               return footLoadPercentage.getDoubleValue();
            }
            
            @Override
            public void computeAndPackFootWrench(Wrench footWrenchToPack)
            {
            }
            
            @Override
            public void computeAndPackCoP(FramePoint2d copToPack)
            {
               logDataProcessorHelper.getMeasuredCoP(robotSide, copToPack);
               copToPack.setIncludingFrame(bipedFeet.get(robotSide).getSoleFrame(), copToPack.getPointCopy());
            }
         };
         
         footSwitches.put(robotSide, footSwitch);
      }
      
      return footSwitches;
   }

   @Override
   public void processDataAtControllerRate()
   {
   }

   private final Vector3d translation = new Vector3d();
   private final Quat4d orientation = new Quat4d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Vector3d linearAcceleration = new Vector3d();
   private final Twist twist = new Twist();
   private final FrameVector linearFrameVelocity = new FrameVector();
   private final FrameVector angularFrameVelocity = new FrameVector();

   private final Matrix3d processedOrientation = new Matrix3d();
   private final Vector3d processedAngularVelocity = new Vector3d();
   private final Vector3d processedLinearAcceleration = new Vector3d();

   @Override
   public void processDataAtStateEstimatorRate()
   {
      logDataProcessorHelper.update();
      for (OneDoFJoint joint : stateEstimatorSensorDefinitions.getJointSensorDefinitions())
      {
         String jointName = joint.getName();
         double q = rawSensorMap.getRawJointPosition(jointName);
         double qd = rawSensorMap.getRawJointVelocity(jointName);
         double tau = rawSensorMap.getRawJointTau(jointName);

         postProcessedSensors.setJointPositionSensorValue(joint, q);
         postProcessedSensors.setJointVelocitySensorValue(joint, qd);
         postProcessedSensors.setJointTauSensorValue(joint, tau);
      }

      for (IMUDefinition imuDefinition : stateEstimatorSensorDefinitions.getIMUSensorDefinitions())
      {
         String imuName = imuDefinition.getName();
         rawSensorMap.getRawIMUOrientation(imuName, orientation);
         rawSensorMap.getRawIMUAngularVelocity(imuName, angularVelocity);
         rawSensorMap.getRawIMULinearAcceleration(imuName, linearAcceleration);

         postProcessedSensors.setOrientationSensorValue(imuDefinition, orientation);
         postProcessedSensors.setAngularVelocitySensorValue(imuDefinition, angularVelocity);
         postProcessedSensors.setLinearAccelerationSensorValue(imuDefinition, linearAcceleration);
      }

      postProcessedSensors.startComputation(rawSensorMap.getTimestamp(), rawSensorMap.getVisionSensorTimestamp(), -1);
      stateEstimator.doControl();

      SixDoFJoint rootJoint = estimatorFullRobotModel.getRootJoint();

      rootJoint.packTranslation(translation);
      estimatedRootJointPosition.set(translation);

      rootJoint.packRotation(orientation);
      estimatedRootJointQuaternion.set(orientation);
      estimatedRootJointOrientation.set(orientation);
      
      rootJoint.packJointTwist(twist);

      twist.packLinearPart(linearFrameVelocity);
      linearFrameVelocity.changeFrame(worldFrame);
      estimatedRootJointLinearVelocity.set(linearFrameVelocity);
      
      twist.packAngularPart(angularFrameVelocity);
      angularFrameVelocity.changeFrame(worldFrame);
      estimatedRootJointAngularVelocity.set(angularFrameVelocity);
      
      for (OneDoFJoint joint : stateEstimatorSensorDefinitions.getJointSensorDefinitions())
      {
         estimatedJointPositionMap.get(joint).set(joint.getQ());
         estimatedJointVelocityMap.get(joint).set(joint.getQd());
      }
      
      List<? extends IMUSensorReadOnly> imuProcessedOutputs = postProcessedSensors.getIMUProcessedOutputs();
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         IMUSensorReadOnly imuProcessedOutput = imuProcessedOutputs.get(i);

         imuProcessedOutput.getOrientationMeasurement(processedOrientation);
         processedIMUOrientationMap.get(i).set(processedOrientation);
         
         imuProcessedOutput.getAngularVelocityMeasurement(processedAngularVelocity);
         processedIMUAngularVelocityMap.get(i).set(processedAngularVelocity);
         
         imuProcessedOutput.getLinearAccelerationMeasurement(processedLinearAcceleration);
         processedIMULinearAccelerationMap.get(i).set(processedLinearAcceleration);
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
