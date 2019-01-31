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
import us.ihmc.ekf.filter.sensor.implementations.FootVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class LeggedRobotEKF implements StateEstimatorController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final YoDouble estimationTime = new YoDouble("EstimationTimeMs", registry);

   private final FloatingJointBasics rootJoint;
   private final PoseState rootState;

   private final List<OneDoFJointBasics> oneDoFJoints;
   private final List<JointState> jointStates = new ArrayList<>();

   private final List<AngularVelocitySensor> angularVelocitySensors = new ArrayList<>();
   private final List<IMUSensorReadOnly> angularVelocitySensorOutputs = new ArrayList<>();
   private final List<LinearAccelerationSensor> linearAccelerationSensors = new ArrayList<>();
   private final List<IMUSensorReadOnly> linearAccelerationSensorOutputs = new ArrayList<>();

   private final SensorRawOutputMapReadOnly sensorOutput;
   private final List<JointPositionSensor> jointPositionSensors = new ArrayList<>();

   private final Wrench footWrench = new Wrench();
   private final double weight;
   private final List<FootVelocitySensor> footVelocitySensors = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> footForces = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> footTorques = new ArrayList<>();
   private final List<ForceSensorDataReadOnly> forceSensorOutputs = new ArrayList<>();
   private final List<ReferenceFrame> copFrames = new ArrayList<>();
   private final FrameVector3D tempVector = new FrameVector3D();

   private final StateEstimator stateEstimator;

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   private final List<YoDouble> jointAngles = new ArrayList<>();
   private final YoFramePose3D rootPose = new YoFramePose3D("RootPose", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoseUsingYawPitchRoll pelvisPoseForVisualization = new YoFramePoseUsingYawPitchRoll("PelvisPose", ReferenceFrame.getWorldFrame(),
                                                                                                            registry);
   private final YoGraphicCoordinateSystem pelvisVisualization = new YoGraphicCoordinateSystem("PelvisFrame", pelvisPoseForVisualization, 0.3,
                                                                                               YoAppearance.Green());

   private final AtomicBoolean fixRobotRequest = new AtomicBoolean(false);
   private final YoBoolean fixRobot = new YoBoolean("FixRobot", registry);

   public LeggedRobotEKF(FloatingJointBasics rootJoint, List<OneDoFJointBasics> oneDoFJoints, String primaryImuName, Collection<String> imuNames,
                         Map<String, ReferenceFrame> forceSensorMap, SensorRawOutputMapReadOnly sensorOutput, double dt, double gravity,
                         Map<String, String> jointGroups, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.sensorOutput = sensorOutput;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;

      weight = -Arrays.asList(ScrewTools.computeSubtreeSuccessors(rootJoint.getPredecessor())).stream().collect(Collectors.summingDouble(body -> body.getInertia().getMass())) * gravity;

      List<Sensor> sensors = new ArrayList<>();
      rootState = createState(rootJoint, oneDoFJoints, dt, sensors, jointGroups);
      createImuSensors(primaryImuName, imuNames, sensorOutput, dt, sensors);
      createFootSensors(rootJoint, forceSensorMap, sensorOutput, dt, gravity, graphicsListRegistry, sensors);

      RobotState robotState = new RobotState(rootState, jointStates);
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      if (graphicsListRegistry != null)
      {
         graphicsListRegistry.registerYoGraphic("EKF", pelvisVisualization);
      }
   }

   private void createFootSensors(FloatingJointBasics rootJoint, Map<String, ReferenceFrame> forceSensorMap, SensorRawOutputMapReadOnly sensorOutput, double dt,
                          double gravity, YoGraphicsListRegistry graphicsListRegistry, List<Sensor> sensors)
   {
      String parameterGroup = "Foot";

      DoubleProvider forceFilter = new DoubleParameter(parameterGroup + "ForceFilter", registry, 100.0);
      DoubleProvider forceAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(forceFilter .getValue(), dt);
      DoubleProvider torqueFilter = new DoubleParameter(parameterGroup + "TorqueFilter", registry, 100.0);
      DoubleProvider torqueAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(torqueFilter .getValue(), dt);

      for (String forceSensorName : forceSensorMap.keySet())
      {
         ReferenceFrame soleFrame = forceSensorMap.get(forceSensorName);
         LogTools.info("Adding foot velocity sensor for " + soleFrame);

         ForceSensorDataReadOnly forceSensorOutput = sensorOutput.getForceSensorRawOutputs().getByName(forceSensorName);
         RigidBodyBasics foot = forceSensorOutput.getMeasurementLink();
         AlphaFilteredYoFrameVector filteredForce = new AlphaFilteredYoFrameVector(foot.getName() + "Force", "", registry, forceAlpha, ReferenceFrame.getWorldFrame());
         AlphaFilteredYoFrameVector filteredTorque = new AlphaFilteredYoFrameVector(foot.getName() + "Torque", "", registry, torqueAlpha, ReferenceFrame.getWorldFrame());
         footForces.add(filteredForce);
         footTorques.add(filteredTorque);

         YoFramePoint3D yoCopPosition = new YoFramePoint3D(soleFrame.getName() + "CopFrame", ReferenceFrame.getWorldFrame(), registry);
         ReferenceFrame copFrame = new ReferenceFrame(soleFrame.getName() + "CopFrame", soleFrame)
         {
            private final FrameVector3D force = new FrameVector3D();
            private final FrameVector3D torque = new FrameVector3D();
            private final FramePoint3D copPosition = new FramePoint3D();

            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               force.setIncludingFrame(filteredForce);
               torque.setIncludingFrame(filteredTorque);
               force.changeFrame(soleFrame);
               torque.changeFrame(soleFrame);

               // If the measured weight is lower then 1% of the robot weight just set the cop to zero.
               if (force.getZ() > weight * 0.01)
               {
                  double copX = -torque.getY() / force.getZ();
                  double copY = torque.getX() / force.getZ();
                  copPosition.setIncludingFrame(getParent(), copX, copY, 0.0);
                  yoCopPosition.setMatchingFrame(copPosition);
               }
               else
               {
                  copPosition.setToZero(getParent());
                  yoCopPosition.setToNaN();
               }

               transformToParent.setTranslationAndIdentityRotation(copPosition);
            }
         };

         if (graphicsListRegistry != null)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(copFrame.getName(), yoCopPosition, 0.02, YoAppearance.Green());
            graphicsListRegistry.registerYoGraphic("EKF", copViz);
         }

         FootVelocitySensor footVelocitySensor = new FootVelocitySensor(dt, foot, copFrame, parameterGroup, registry);
         footVelocitySensors.add(footVelocitySensor);
         forceSensorOutputs.add(forceSensorOutput);
         copFrames.add(copFrame);
         sensors.add(footVelocitySensor);
      }
   }

   private void createImuSensors(String primaryImuName, Collection<String> imuNames, SensorRawOutputMapReadOnly sensorOutput, double dt, List<Sensor> sensors)
   {
      List<IMUSensorReadOnly> imuOutputs = sensorOutput.getIMURawOutputs().stream().filter(imu -> imuNames.contains(imu.getSensorName())).collect(Collectors.toList());
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
            LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor(name + "LinearAcceleration", dt, imuBody, imuFrame, false, registry);
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
         JointPositionSensor jointPositionSensor = new JointPositionSensorWithBacklash(jointName, parameterGroup, dt, registry);
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

      estimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));

      updateYoVariables();
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

      pelvisPoseForVisualization.set(rootPose);
   }

   private void updateSensors()
   {
      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double jointPositionMeasurement = sensorOutput.getJointPositionRawOutput(oneDoFJoints.get(jointIdx));
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
         forceSensorOutputs.get(footIdx).getWrench(footWrench);

         tempVector.setIncludingFrame(footWrench.getLinearPart());
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         footForces.get(footIdx).update(tempVector);

         tempVector.setIncludingFrame(footWrench.getAngularPart());
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         footTorques.get(footIdx).update(tempVector);

         copFrames.get(footIdx).update();

         // When fixing the robot this will cause the state estimator to assume the feet are not moving:
         double loadPercentage = fixRobot.getValue() ? 1.0 : footForces.get(footIdx).getZ() / weight;
         footVelocitySensors.get(footIdx).setLoad(loadPercentage);
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
