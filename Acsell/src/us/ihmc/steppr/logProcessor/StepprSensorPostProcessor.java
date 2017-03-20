package us.ihmc.steppr.logProcessor;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.LogDataProcessorFunction;
import us.ihmc.avatar.logProcessor.LogDataProcessorHelper;
import us.ihmc.avatar.logProcessor.LogDataRawSensorMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
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
            DoubleYoVariable maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", 0.1);
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

            sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, maxDeflection, false);

            sensorProcessing.computeJointVelocityFromFiniteDifference(jointVelocityAlphaFilter, true); //vizonly
            sensorProcessing.addSensorAlphaFilter(jointVelocityAlphaFilter, false, JOINT_VELOCITY);
            sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

            sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
            sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);
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
      SideDependentList<? extends ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(estimatorFullRobotModel, referenceFrames);
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // no viz for now
      
      Map<RigidBody, ContactablePlaneBody> bipedFeetMap = new HashMap<>();
      Map<RigidBody, FootSwitchInterface> footSwitchMap = createStateEstimatorFootSwitches(logDataProcessorHelper, bipedFeet);
      final Map<RigidBody, RobotSide> feetMap = new LinkedHashMap<>();
      Map<RigidBody, ReferenceFrame> soleFrames = new LinkedHashMap<RigidBody, ReferenceFrame>();
      
      for(RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = bipedFeet.get(robotSide);
         RigidBody footRigidBody = contactableFoot.getRigidBody();
         bipedFeetMap.put(footRigidBody, contactableFoot);
         feetMap.put(footRigidBody, robotSide);
         soleFrames.put(footRigidBody, referenceFrames.getSoleFrame(robotSide));
      }

      CenterOfPressureDataHolder centerOfPressureDataHolder = new CenterOfPressureDataHolder(soleFrames)
      {
         @Override
         public void getCenterOfPressure(FramePoint2d centerOfPressureToPack, RigidBody foot)
         {
            logDataProcessorHelper.getDesiredCoP(feetMap.get(foot), centerOfPressureToPack);
         }
      };
      
      CenterOfMassDataHolder centerOfMassDataHolderToUpdate = new CenterOfMassDataHolder();

      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            postProcessedSensors, null, centerOfMassDataHolderToUpdate,
            imuSensorsToUseInStateEstimator, gravitationalAcceleration, footSwitchMap, centerOfPressureDataHolder, new RobotMotionStatusHolder(), bipedFeetMap, yoGraphicsListRegistry);
      return stateEstimator;
   }

   private Map<RigidBody, FootSwitchInterface> createStateEstimatorFootSwitches(final LogDataProcessorHelper logDataProcessorHelper, final SideDependentList<? extends ContactablePlaneBody> bipedFeet)
   {
      Map<RigidBody, FootSwitchInterface> footSwitches = new HashMap<>();
      
      for (final RobotSide robotSide : RobotSide.values)
      {
         final ContactablePlaneBody contactableFoot = bipedFeet.get(robotSide);
         String namePrefix = contactableFoot.getName() + "StateEstimator";
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
               copToPack.setIncludingFrame(contactableFoot.getSoleFrame(), copToPack.getPointCopy());
            }

            @Override
            public void updateCoP()
            {
            }

            @Override
            @Deprecated
            public void setFootContactState(boolean hasFootHitGround)
            {
               // TODO Auto-generated method stub
               
            }

            @Override
            public void trustFootSwitch(boolean trustFootSwitch)
            {
            }
         };
         
         footSwitches.put(contactableFoot.getRigidBody(), footSwitch);
      }
      
      return footSwitches;
   }

   @Override
   public void processDataAtControllerRate()
   {
   }

   private final Vector3D translation = new Vector3D();
   private final Quaternion orientation = new Quaternion();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   private final Twist twist = new Twist();
   private final FrameVector linearFrameVelocity = new FrameVector();
   private final FrameVector angularFrameVelocity = new FrameVector();

   private final RotationMatrix processedOrientation = new RotationMatrix();
   private final Vector3D processedAngularVelocity = new Vector3D();
   private final Vector3D processedLinearAcceleration = new Vector3D();

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

      FloatingInverseDynamicsJoint rootJoint = estimatorFullRobotModel.getRootJoint();

      rootJoint.getTranslation(translation);
      estimatedRootJointPosition.set(translation);

      rootJoint.getRotation(orientation);
      estimatedRootJointQuaternion.set(orientation);
      estimatedRootJointOrientation.set(orientation);
      
      rootJoint.getJointTwist(twist);

      twist.getLinearPart(linearFrameVelocity);
      linearFrameVelocity.changeFrame(worldFrame);
      estimatedRootJointLinearVelocity.set(linearFrameVelocity);
      
      twist.getAngularPart(angularFrameVelocity);
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
