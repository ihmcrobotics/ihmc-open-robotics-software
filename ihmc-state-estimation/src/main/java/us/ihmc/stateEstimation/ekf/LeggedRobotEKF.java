package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
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
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class LeggedRobotEKF implements StateEstimatorController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final YoDouble estimationTime = new YoDouble("EstimationTimeMs", registry);

   private final FloatingInverseDynamicsJoint rootJoint;
   private final PoseState rootState;

   private final List<OneDoFJoint> oneDoFJoints;
   private final List<JointState> jointStates = new ArrayList<>();

   private final List<IMUSensorReadOnly> imuOutputs;
   private final List<AngularVelocitySensor> angularVelocitySensors = new ArrayList<>();
   private final List<LinearAccelerationSensor> linearAccelerationSensors = new ArrayList<>();

   private final SensorRawOutputMapReadOnly sensorOutput;
   private final List<JointPositionSensor> jointPositionSensors = new ArrayList<>();

   private final Wrench footWrench = new Wrench();
   private final FrameVector3D footForce = new FrameVector3D();
   private final double weight;
   private final List<FootVelocitySensor> footVelocitySensors = new ArrayList<>();
   private final List<ForceSensorDataReadOnly> forceSensorOutputs = new ArrayList<>();
   private final List<ReferenceFrame> copFrames = new ArrayList<>();

   private final StateEstimator stateEstimator;

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   private final List<YoDouble> jointAngles = new ArrayList<>();
   private final YoFramePose3D rootPose = new YoFramePose3D("RootPose", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoseUsingYawPitchRoll pelvisPoseForVisualization = new YoFramePoseUsingYawPitchRoll("PelvisPose", ReferenceFrame.getWorldFrame(),
                                                                                                            registry);
   private final YoGraphicCoordinateSystem pelvisVisualization = new YoGraphicCoordinateSystem("PelvisFrame", pelvisPoseForVisualization, 0.3,
                                                                                               YoAppearance.Green());

   private boolean isInitialized = false;

   public LeggedRobotEKF(FloatingInverseDynamicsJoint rootJoint, List<OneDoFJoint> oneDoFJoints, Collection<String> imuNames,
                         Map<String, ReferenceFrame> forceSensorMap, SensorRawOutputMapReadOnly sensorOutput, double dt, double gravity,
                         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.sensorOutput = sensorOutput;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;

      String rootBodyName = rootJoint.getSuccessor().getName();
      PrintTools.info("Creating pose state for " + rootBodyName);
      rootState = new PoseState(rootBodyName, dt, rootJoint.getFrameAfterJoint(), registry);
      List<Sensor> sensors = new ArrayList<>();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         PrintTools.info("Creating joint state for " + oneDoFJoint.getName());
         JointState jointState = new JointState(oneDoFJoint.getName(), dt, registry);
         jointStates.add(jointState);
         JointPositionSensor jointPositionSensor = new JointPositionSensor(oneDoFJoint.getName(), dt, registry);
         jointPositionSensors.add(jointPositionSensor);
         sensors.add(jointPositionSensor);
         jointAngles.add(new YoDouble(oneDoFJoint.getName() + "JointAngle", registry));
      }

      imuOutputs = sensorOutput.getIMURawOutputs().stream().filter(imu -> imuNames.contains(imu.getSensorName())).collect(Collectors.toList());
      for (IMUSensorReadOnly imuOutput : imuOutputs)
      {
         PrintTools.info("Adding IMU sensor " + imuOutput.getSensorName());
         String name = FilterTools.stringToPrefix(imuOutput.getSensorName());
         RigidBody imuBody = imuOutput.getMeasurementLink();
         ReferenceFrame imuFrame = imuOutput.getMeasurementFrame();
         AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor(name + "AngularVelocity", dt, imuBody, imuFrame, true, registry);
         LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor(name + "LinearAcceleration", dt, imuBody, imuFrame, true, registry);
         angularVelocitySensors.add(angularVelocitySensor);
         linearAccelerationSensors.add(linearAccelerationSensor);
         sensors.add(angularVelocitySensor);
         sensors.add(linearAccelerationSensor);
      }

      weight = -Arrays.asList(ScrewTools.computeSubtreeSuccessors(rootJoint.getPredecessor())).stream().collect(Collectors.summingDouble(body -> body.getInertia().getMass())) * gravity;
      for (String forceSensorName : forceSensorMap.keySet())
      {
         ReferenceFrame soleFrame = forceSensorMap.get(forceSensorName);
         PrintTools.info("Adding foot velocity sensor for " + soleFrame);

         ForceSensorDataReadOnly forceSensorOutput = sensorOutput.getForceSensorRawOutputs().getByName(forceSensorName);
         YoFramePoint3D yoCopPosition = new YoFramePoint3D(soleFrame.getName() + "CopFrame", ReferenceFrame.getWorldFrame(), registry);
         ReferenceFrame copFrame = new ReferenceFrame(soleFrame.getName() + "CopFrame", soleFrame)
         {
            private final Wrench wrench = new Wrench();
            private final FramePoint3D copPosition = new FramePoint3D();

            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               forceSensorOutput.getWrench(wrench);
               wrench.changeFrame(soleFrame);

               // If the measured weight is lower then 1% of the robot weight just set the cop to zero.
               if (wrench.getLinearPartZ() > weight * 0.01)
               {
                  double copX = -wrench.getAngularPartY() / wrench.getLinearPartZ();
                  double copY = wrench.getAngularPartX() / wrench.getLinearPartZ();
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

         RigidBody foot = forceSensorOutput.getMeasurementLink();
         FootVelocitySensor footVelocitySensor = new FootVelocitySensor(dt, foot, copFrame, registry);
         footVelocitySensors.add(footVelocitySensor);
         forceSensorOutputs.add(forceSensorOutput);
         copFrames.add(copFrame);
         sensors.add(footVelocitySensor);
      }

      RobotState robotState = new RobotState(rootState, jointStates);
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      if (graphicsListRegistry != null)
      {
         graphicsListRegistry.registerYoGraphic("EKF", pelvisVisualization);
      }
   }

   @Override
   public void doControl()
   {
      if (!isInitialized)
      {
         initializeJointAngles();
      }

      long startTime = System.nanoTime();

      stateEstimator.predict();
      updateRobot();

      updateSensors();
      stateEstimator.correct();
      updateRobot();

      estimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));

      updateYoVariables();
   }

   private void initializeJointAngles()
   {
      // TODO: remove this
      // on the first tick this sets all joint positions to the measured value this should really be done through a proper initialization or by increasing the initial joint position uncertainty.
      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         double jointPositionMeasurement = sensorOutput.getJointPositionRawOutput(oneDoFJoints.get(jointIdx));
         JointState jointState = jointStates.get(jointIdx);
         jointState.initialize(jointPositionMeasurement, 0.0);
      }
      isInitialized = true;
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
      for (int imuIdx = 0; imuIdx < imuOutputs.size(); imuIdx++)
      {
         IMUSensorReadOnly imuOutput = imuOutputs.get(imuIdx);
         angularVelocitySensors.get(imuIdx).setMeasurement(imuOutput.getAngularVelocityMeasurement());
         linearAccelerationSensors.get(imuIdx).setMeasurement(imuOutput.getLinearAccelerationMeasurement());
      }
      for (int footIdx = 0; footIdx < footVelocitySensors.size(); footIdx++)
      {
         copFrames.get(footIdx).update();
         forceSensorOutputs.get(footIdx).getWrench(footWrench);
         footWrench.getLinearPartIncludingFrame(footForce);
         footForce.changeFrame(ReferenceFrame.getWorldFrame());
         footVelocitySensors.get(footIdx).setLoad(footForce.getZ() / weight);
      }
   }

   private void updateRobot()
   {
      rootState.getTransform(rootTransform);
      rootState.getTwist(rootTwist);

      rootJoint.setPositionAndRotation(rootTransform);
      rootJoint.setJointTwist(rootTwist);

      for (int jointIdx = 0; jointIdx < oneDoFJoints.size(); jointIdx++)
      {
         JointState jointState = jointStates.get(jointIdx);
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(jointIdx);
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
