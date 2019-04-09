package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.FilterTools.ProccessNoiseModel;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.implementations.JointVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * A state estimator implementation for legged robots:</br>
 * - Creates one pose state and joint states</br>
 * - Creates joint position sensors for one DoF joints</br>
 * - Creates angular velocity sensors for IMUs</br>
 * - Creates linear acceleration sensors for primary IMUs</br>
 * - Creates foot linear velocity sensors for feet in contact</br>
 * <p>
 * This class sets up the estimator and wraps it in the {@link StateEstimatorController} interface.
 *
 * @author Georg Wiedebach
 */
public class LeggedRobotEKF implements StateEstimatorController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final YoDouble estimationTime = new YoDouble("EstimationTimeMs", registry);

   private final FloatingJointBasics rootJoint;
   private final PoseState rootState;
   private final List<OneDoFJointBasics> oneDoFJoints;
   /**
    * These are needed for now since the estimator runs on its own copy of the full robot model. The reference joints
    * are used to get the sensor measurements.
    */
   private final List<OneDoFJointBasics> referenceJoints;
   private final List<JointState> jointStates = new ArrayList<>();

   private final List<AngularVelocitySensor> angularVelocitySensors = new ArrayList<>();
   private final List<IMUSensorReadOnly> angularVelocitySensorOutputs = new ArrayList<>();
   private final List<LinearAccelerationSensor> linearAccelerationSensors = new ArrayList<>();
   private final List<IMUSensorReadOnly> linearAccelerationSensorOutputs = new ArrayList<>();
   private final SensorOutputMapReadOnly processedSensorOutput;
   private final List<JointPositionSensor> jointPositionSensors = new ArrayList<>();
   private final List<JointVelocitySensor> jointVelocitySensors = new ArrayList<>();
   private final List<FootWrenchSensorUpdater> footWrenchSensorUpdaters = new ArrayList<>();
   private final List<ForceSensorDataReadOnly> forceSensorOutputs = new ArrayList<>();
   private final List<ReferenceFrame> forceSensorMeasurementFrames = new ArrayList<>();

   private final StateEstimator stateEstimator;

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   private final List<YoDouble> yoJointAngles = new ArrayList<>();
   private final List<YoDouble> yoJointVelocities = new ArrayList<>();
   private final YoFramePose3D yoRootPose;
   private final YoFixedFrameTwist yoRootTwist;
   private final YoFrameVector3D linearVelocityInWorld = new YoFrameVector3D("RootLinearVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoseUsingYawPitchRoll pelvisPoseViz = new YoFramePoseUsingYawPitchRoll("PelvisPose", ReferenceFrame.getWorldFrame(), registry);

   private final AtomicBoolean fixRobotRequest = new AtomicBoolean(false);
   private final YoBoolean fixRobot = new YoBoolean("FixRobot", registry);

   public LeggedRobotEKF(FloatingJointBasics rootJoint, List<OneDoFJointBasics> oneDoFJoints, String primaryImuName, Map<String, IMUDefinition> imuSensorMap,
                         Map<String, ImmutablePair<ReferenceFrame, ForceSensorDefinition>> forceSensorMap, SensorRawOutputMapReadOnly sensorOutput,
                         SensorOutputMapReadOnly processedSensorOutput, double dt, double gravity, Map<String, String> jointGroups,
                         YoGraphicsListRegistry graphicsListRegistry, List<OneDoFJointBasics> referenceJoints)
   {
      FilterTools.proccessNoiseModel = ProccessNoiseModel.ONLY_ACCELERATION_VARIANCE;

      this.processedSensorOutput = processedSensorOutput;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
      this.referenceJoints = referenceJoints;

      List<Sensor> sensors = new ArrayList<>();
      rootState = createState(rootJoint, oneDoFJoints, dt, sensors, jointGroups);
      createImuSensors(primaryImuName, imuSensorMap, processedSensorOutput, dt, sensors);
      createFootSensors(rootJoint, forceSensorMap, processedSensorOutput, dt, gravity, graphicsListRegistry, sensors);

      RobotState robotState = new RobotState(rootState, jointStates);
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      yoRootPose = new YoFramePose3D("RootPoseEKF", ReferenceFrame.getWorldFrame(), registry);
      yoRootTwist = new YoFixedFrameTwist("RootTwistEKF", rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                                          registry);

      if (graphicsListRegistry != null)
      {
         graphicsListRegistry.registerYoGraphic("EKF", new YoGraphicCoordinateSystem("PelvisFrame", pelvisPoseViz, 0.3, YoAppearance.Green()));
      }
   }

   private void createFootSensors(FloatingJointBasics rootJoint, Map<String, ImmutablePair<ReferenceFrame, ForceSensorDefinition>> forceSensorMap,
                                  SensorOutputMapReadOnly sensorOutput, double dt, double gravity, YoGraphicsListRegistry graphicsListRegistry,
                                  List<Sensor> sensors)
   {
      RigidBodyBasics[] allBodies = ScrewTools.computeSubtreeSuccessors(rootJoint.getPredecessor());
      Double mass = Arrays.asList(allBodies).stream().collect(Collectors.summingDouble(body -> body.getInertia().getMass()));
      double weight = -mass * gravity;

      for (String forceSensorName : forceSensorMap.keySet())
      {
         ForceSensorDataReadOnly forceSensorOutput = sensorOutput.getForceSensorProcessedOutputs().getByName(forceSensorName);
         ReferenceFrame soleFrame = forceSensorMap.get(forceSensorName).getLeft();
         ReferenceFrame measurementFrame = forceSensorMap.get(forceSensorName).getRight().getSensorFrame();
         RigidBodyBasics foot = forceSensorMap.get(forceSensorName).getRight().getRigidBody();

         LogTools.info("Adding foot velocity sensor for " + soleFrame);
         FootWrenchSensorUpdater footWrenchSensorUpdater = new FootWrenchSensorUpdater(foot, soleFrame, dt, weight, graphicsListRegistry, registry);

         footWrenchSensorUpdaters.add(footWrenchSensorUpdater);
         forceSensorOutputs.add(forceSensorOutput);
         forceSensorMeasurementFrames.add(measurementFrame);
         sensors.add(footWrenchSensorUpdater.getFootLinearVelocitySensor());
      }
   }

   private void createImuSensors(String primaryImuName, Map<String, IMUDefinition> imuSensorMap, SensorOutputMapReadOnly sensorOutput, double dt,
                                 List<Sensor> sensors)
   {
      for (String imuName : imuSensorMap.keySet())
      {
         String name = FilterTools.stringToPrefix(imuName);
         RigidBodyBasics imuBody = imuSensorMap.get(imuName).getRigidBody();
         ReferenceFrame imuFrame = imuSensorMap.get(imuName).getIMUFrame();

         IMUSensorReadOnly imuOutput = sensorOutput.getIMUProcessedOutputs().stream().filter(imu -> imu.getSensorName().equals(imuName)).findFirst().get();
         LogTools.info("Adding angular velocity sensor " + imuName);
         AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor(name + "AngularVelocity", dt, imuBody, imuFrame, false, registry);
         angularVelocitySensors.add(angularVelocitySensor);
         angularVelocitySensorOutputs.add(imuOutput);
         sensors.add(angularVelocitySensor);

         if (imuName.equals(primaryImuName))
         {
            LogTools.info("Adding linear acceleration sensor " + imuName);
            LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor(name + "LinearAcceleration", dt, imuBody, imuFrame, false,
                                                                                             registry);
            linearAccelerationSensors.add(linearAccelerationSensor);
            linearAccelerationSensorOutputs.add(imuOutput);
            sensors.add(linearAccelerationSensor);
         }
      }
   }

   private PoseState createState(FloatingJointBasics rootJoint, List<OneDoFJointBasics> oneDoFJoints, double dt, List<Sensor> sensors,
                                 Map<String, String> jointGroups)
   {
      String rootBodyName = rootJoint.getSuccessor().getName();
      LogTools.info("Creating pose state for " + rootBodyName);
      PoseState rootState = new PoseState(rootBodyName, dt, rootJoint.getFrameAfterJoint(), registry);
      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         String jointName = oneDoFJoint.getName();
         String parameterGroup = FilterTools.stringToPrefix(jointGroups.containsKey(jointName) ? jointGroups.get(jointName) : jointName);
         LogTools.info("Creating joint state for " + jointName + " in group " + parameterGroup);
         JointState jointState = new JointState(jointName, parameterGroup, dt, registry);
         jointStates.add(jointState);
         JointPositionSensor jointPositionSensor = new JointPositionSensor(jointName, parameterGroup, dt, registry);
         jointPositionSensors.add(jointPositionSensor);
         sensors.add(jointPositionSensor);
         JointVelocitySensor jointVelocitySensor = new JointVelocitySensor(jointName, parameterGroup, dt, registry);
         jointVelocitySensors.add(jointVelocitySensor);
         sensors.add(jointVelocitySensor);
         yoJointAngles.add(new YoDouble("q_" + jointName + "_ekf", registry));
         yoJointVelocities.add(new YoDouble("qd_" + jointName + "_ekf", registry));
      }
      return rootState;
   }

   @Override
   public void doControl()
   {
      long startTime = System.nanoTime();

      fixRobot.set(fixRobotRequest.get());

      stateEstimator.predict();
      updateRobot();

      updateSensors();
      stateEstimator.correct();
      updateRobot();

      updateYoVariables();

      estimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   private void updateYoVariables()
   {
      rootState.getTransform(rootTransform);
      rootState.getTwist(rootTwist);

      yoRootPose.set(rootTransform);
      yoRootTwist.set(rootTwist);
      linearVelocityInWorld.setMatchingFrame(rootTwist.getLinearPart());

      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         JointState jointState = jointStates.get(jointIdx);
         yoJointAngles.get(jointIdx).set(jointState.getQ());
         yoJointVelocities.get(jointIdx).set(jointState.getQd());
      }

      pelvisPoseViz.set(yoRootPose);
   }

   private final Wrench tempWrench = new Wrench();

   private void updateSensors()
   {
      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double jointPositionMeasurement = processedSensorOutput.getJointPositionProcessedOutput(referenceJoints.get(jointIdx));
         jointPositionSensors.get(jointIdx).setJointPositionMeasurement(jointPositionMeasurement);
         double jointVelocityMeasurement = processedSensorOutput.getJointVelocityProcessedOutput(referenceJoints.get(jointIdx));
         jointVelocitySensors.get(jointIdx).setJointVelocityMeasurement(jointVelocityMeasurement);
      }

      for (int imuIdx = 0; imuIdx < angularVelocitySensors.size(); imuIdx++)
      {
         Vector3DReadOnly angularVelocityMeasurement = angularVelocitySensorOutputs.get(imuIdx).getAngularVelocityMeasurement();
         angularVelocitySensors.get(imuIdx).setMeasurement(angularVelocityMeasurement);
      }

      for (int imuIdx = 0; imuIdx < linearAccelerationSensors.size(); imuIdx++)
      {
         Vector3DReadOnly linearAccelerationMeasurement = linearAccelerationSensorOutputs.get(imuIdx).getLinearAccelerationMeasurement();
         linearAccelerationSensors.get(imuIdx).setMeasurement(linearAccelerationMeasurement);
      }

      for (int footIdx = 0; footIdx < footWrenchSensorUpdaters.size(); footIdx++)
      {
         forceSensorOutputs.get(footIdx).getWrench(tempWrench);
         tempWrench.setReferenceFrame(forceSensorMeasurementFrames.get(footIdx));
         footWrenchSensorUpdaters.get(footIdx).update(tempWrench, fixRobot.getValue());
      }
   }

   private void updateRobot()
   {
      rootState.getTransform(rootTransform);
      rootState.getTwist(rootTwist);

      rootJoint.setJointConfiguration(rootTransform);
      rootJoint.setJointTwist(rootTwist);

      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         JointState jointState = jointStates.get(jointIdx);
         OneDoFJointBasics oneDoFJoint = oneDoFJoints.get(jointIdx);
         oneDoFJoint.setQ(jointState.getQ());
         oneDoFJoint.setQd(jointState.getQd());
      }

      rootJoint.updateFramesRecursively();
   }

   @Override
   public void initializeEstimator(RigidBodyTransform rootJointTransform)
   {
      rootTwist.setToZero(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      rootState.initialize(rootJointTransform, rootTwist);

      stateEstimator.reset();
      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {
         linearAccelerationSensors.get(i).resetBias();
      }
      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         angularVelocitySensors.get(i).resetBias();
      }
   }

   @Override
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double initialJointPosition = jointPositions.get(oneDoFJoints.get(jointIdx).getName());
         JointState jointState = jointStates.get(jointIdx);
         jointState.initialize(initialJointPosition, 0.0);
      }

      initializeEstimator(rootJointTransform);
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      fixRobotRequest.set(operatingMode == StateEstimatorMode.FROZEN);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
