package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
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
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

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
   private final List<FootWrenchSensor> footVelocitySensors = new ArrayList<>();
   private final List<ForceSensorDataReadOnly> forceSensorOutputs = new ArrayList<>();

   private final StateEstimator stateEstimator;

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   private final List<YoDouble> jointAngles = new ArrayList<>();
   private final YoFramePose3D rootPose = new YoFramePose3D("RootPose", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoseUsingYawPitchRoll pelvisPoseViz = new YoFramePoseUsingYawPitchRoll("PelvisPose", ReferenceFrame.getWorldFrame(), registry);

   private final AtomicBoolean fixRobotRequest = new AtomicBoolean(false);
   private final YoBoolean fixRobot = new YoBoolean("FixRobot", registry);

   public LeggedRobotEKF(FloatingJointBasics rootJoint, List<OneDoFJointBasics> oneDoFJoints, String primaryImuName, Collection<String> imuNames,
                         Map<String, ReferenceFrame> forceSensorMap, SensorRawOutputMapReadOnly sensorOutput, SensorOutputMapReadOnly processedSensorOutput,
                         double dt, double gravity, Map<String, String> jointGroups, YoGraphicsListRegistry graphicsListRegistry,
                         List<OneDoFJointBasics> referenceJoints)
   {
      this.processedSensorOutput = processedSensorOutput;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
      this.referenceJoints = referenceJoints;

      List<Sensor> sensors = new ArrayList<>();
      rootState = createState(rootJoint, oneDoFJoints, dt, sensors, jointGroups);
      createImuSensors(primaryImuName, imuNames, processedSensorOutput, dt, sensors);
      createFootSensors(rootJoint, forceSensorMap, processedSensorOutput, dt, gravity, graphicsListRegistry, sensors);

      RobotState robotState = new RobotState(rootState, jointStates);
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      if (graphicsListRegistry != null)
      {
         graphicsListRegistry.registerYoGraphic("EKF", new YoGraphicCoordinateSystem("PelvisFrame", pelvisPoseViz, 0.3, YoAppearance.Green()));
      }
   }

   private void createFootSensors(FloatingJointBasics rootJoint, Map<String, ReferenceFrame> forceSensorMap, SensorOutputMapReadOnly sensorOutput, double dt,
                                  double gravity, YoGraphicsListRegistry graphicsListRegistry, List<Sensor> sensors)
   {
      RigidBodyBasics[] allBodies = ScrewTools.computeSubtreeSuccessors(rootJoint.getPredecessor());
      Double mass = Arrays.asList(allBodies).stream().collect(Collectors.summingDouble(body -> body.getInertia().getMass()));
      double weight = -mass * gravity;

      for (String forceSensorName : forceSensorMap.keySet())
      {
         ForceSensorDataReadOnly forceSensorOutput = sensorOutput.getForceSensorProcessedOutputs().getByName(forceSensorName);
         ReferenceFrame soleFrame = forceSensorMap.get(forceSensorName);
         RigidBodyBasics foot = forceSensorOutput.getMeasurementLink();

         LogTools.info("Adding foot velocity sensor for " + soleFrame);
         FootWrenchSensor footWrenchSensor = new FootWrenchSensor(foot, soleFrame, dt, weight, graphicsListRegistry, registry);

         footVelocitySensors.add(footWrenchSensor);
         forceSensorOutputs.add(forceSensorOutput);
         sensors.add(footWrenchSensor.getSensor());
      }
   }

   private void createImuSensors(String primaryImuName, Collection<String> imuNames, SensorOutputMapReadOnly sensorOutput, double dt, List<Sensor> sensors)
   {
      List<IMUSensorReadOnly> imuOutputs = sensorOutput.getIMUProcessedOutputs().stream().filter(imu -> imuNames.contains(imu.getSensorName()))
                                                       .collect(Collectors.toList());
      for (IMUSensorReadOnly imuOutput : imuOutputs)
      {
         String name = FilterTools.stringToPrefix(imuOutput.getSensorName());
         RigidBodyBasics imuBody = imuOutput.getMeasurementLink();
         ReferenceFrame imuFrame = imuOutput.getMeasurementFrame();

         LogTools.info("Adding angular velocity sensor " + imuOutput.getSensorName());
         AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor(name + "AngularVelocity", dt, imuBody, imuFrame, false, registry);
         angularVelocitySensors.add(angularVelocitySensor);
         angularVelocitySensorOutputs.add(imuOutput);
         sensors.add(angularVelocitySensor);

         if (imuOutput.getSensorName().equals(primaryImuName))
         {
            LogTools.info("Adding linear acceleration sensor " + imuOutput.getSensorName());
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
         LogTools.info("Creating joint state for " + jointName);
         String parameterGroup = FilterTools.stringToPrefix(jointGroups.containsKey(jointName) ? jointGroups.get(jointName) : jointName);
         JointState jointState = new JointState(jointName, parameterGroup, dt, registry);
         jointStates.add(jointState);
         JointPositionSensor jointPositionSensor = new JointPositionSensor(jointName, parameterGroup, dt, registry);
         jointPositionSensors.add(jointPositionSensor);
         sensors.add(jointPositionSensor);
         jointAngles.add(new YoDouble(jointName + "JointAngle", registry));
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
      rootPose.set(rootTransform);

      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         JointState jointState = jointStates.get(jointIdx);
         jointAngles.get(jointIdx).set(jointState.getQ());
      }

      pelvisPoseViz.set(rootPose);
   }

   private final Wrench tempWrench = new Wrench();

   private void updateSensors()
   {
      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double jointPositionMeasurement = processedSensorOutput.getJointPositionProcessedOutput(referenceJoints.get(jointIdx));
         jointPositionSensors.get(jointIdx).setJointPositionMeasurement(jointPositionMeasurement);
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

      for (int footIdx = 0; footIdx < footVelocitySensors.size(); footIdx++)
      {
         forceSensorOutputs.get(footIdx).getWrench(tempWrench);
         footVelocitySensors.get(footIdx).update(tempWrench, fixRobot.getValue());
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
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      rootTwist.setToZero(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      rootState.initialize(rootJointTransform, rootTwist);

      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double initialJointPosition = jointPositions.get(oneDoFJoints.get(jointIdx).getName());
         JointState jointState = jointStates.get(jointIdx);
         jointState.initialize(initialJointPosition, 0.0);
      }
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
