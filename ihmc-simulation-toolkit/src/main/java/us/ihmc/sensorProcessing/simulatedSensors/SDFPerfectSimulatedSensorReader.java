package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.*;
import java.util.Map.Entry;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SDFPerfectSimulatedSensorReader implements RawSensorReader, SensorOutputMapReadOnly
{
   private final String name;
   private final FloatingRootJointRobot robot;
   private final FloatingJointBasics rootJoint;
   private final ReferenceFrames referenceFrames;

   private final List<OneDoFJointStateReadOnly> jointSensorOutputList = new ArrayList<>();
   private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointToSensorOutputMap = new HashMap<>();
   private final List<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>> oneDoFJointPairs = new ArrayList<>();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<>();
   private final List<IMUSensor> imuSensors = new ArrayList<>();

   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;

   public SDFPerfectSimulatedSensorReader(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames)
   {
      this(robot, fullRobotModel, null, referenceFrames);
   }

   public SDFPerfectSimulatedSensorReader(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolderToUpdate,
                                          ReferenceFrames referenceFrames)
   {
      this(robot, fullRobotModel.getRootJoint(), forceSensorDataHolderToUpdate, referenceFrames);
   }

   public SDFPerfectSimulatedSensorReader(FloatingRootJointRobot robot, FloatingJointBasics rootJoint, ForceSensorDataHolder forceSensorDataHolderToUpdate,
                                          ReferenceFrames referenceFrames)
   {
      name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.referenceFrames = referenceFrames;
      this.forceSensorDataHolderToUpdate = forceSensorDataHolderToUpdate;

      this.rootJoint = rootJoint;

      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            String name = oneDoFJoint.getName();
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robot.getOneDegreeOfFreedomJoint(name);

            ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = new ImmutablePair<>(oneDegreeOfFreedomJoint, oneDoFJoint);
            oneDoFJointPairs.add(jointPair);
            OneDoFJointStateReadOnly jointSensorOutput = OneDoFJointStateReadOnly.createFromOneDoFJoint(oneDoFJoint, true);
            jointSensorOutputList.add(jointSensorOutput);
            jointToSensorOutputMap.put(oneDoFJoint, jointSensorOutput);
         }
      }
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.put(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }

   public void addIMUSensor(IMUDefinition imuDefinition)
   {
      imuSensors.add(new IMUSensor(imuDefinition, null));
   }

   @Override
   public void initialize()
   {
      read();
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

   private final RigidBodyTransform temporaryRootToWorldTransform = new RigidBodyTransform();

   @Override
   public void read()
   {
      // Think about adding root body acceleration to the fullrobotmodel
      readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations();
      readAndUpdateRootJointPositionAndOrientation();
      readAndUpdateRootJointAngularAndLinearVelocity();
      // Update frames after setting angular and linear velocities to correctly update zup frames
      updateReferenceFrames();

      long timestamp = Conversions.secondsToNanoseconds(robot.getTime());
      this.timestamp.set(timestamp);
      controllerTimestamp.set(timestamp);
      sensorHeadPPSTimetamp.set(timestamp);

      if (forceSensorDataHolderToUpdate != null)
      {
         for (Entry<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensorEntry : forceTorqueSensors.entrySet())
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensorEntry.getValue();
            forceTorqueSensor.calculate();
            forceSensorDataHolderToUpdate.setForceSensorValue(forceTorqueSensorEntry.getKey(), forceTorqueSensor.getWrench());
         }
      }

      for (IMUSensor imuSensor : imuSensors)
      {
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();
         Twist twist = new Twist(imuSensor.getMeasurementLink().getBodyFixedFrame().getTwistOfFrame());
         twist.changeFrame(measurementFrame);
         imuSensor.setOrientationMeasurement(measurementFrame.getTransformToRoot().getRotation());
         imuSensor.setAngularVelocityMeasurement(twist.getAngularPart());
         // TODO Add acceleration update.
      }
   }

   private void readAndUpdateRootJointAngularAndLinearVelocity()
   {
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame pelvisFrame = rootJoint.getFrameAfterJoint();

      // Update base frames without updating all frames to transform velocity into pelvis
      elevatorFrame.update();
      pelvisFrame.update();

      FrameVector3D linearVelocity = robot.getRootJointVelocity();
      linearVelocity.changeFrame(pelvisFrame);

      FrameVector3D angularVelocity = robot.getRootJointAngularVelocityInRootJointFrame(pelvisFrame);
      angularVelocity.changeFrame(pelvisFrame);

      Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, angularVelocity, linearVelocity);
      rootJoint.setJointTwist(bodyTwist);
   }

   private void updateReferenceFrames()
   {
      if (referenceFrames != null)
      {
         referenceFrames.updateFrames();
      }
      else
      {
         rootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   private void readAndUpdateRootJointPositionAndOrientation()
   {
      packRootTransform(robot, temporaryRootToWorldTransform);
      temporaryRootToWorldTransform.getRotation().normalize();
      rootJoint.setJointConfiguration(temporaryRootToWorldTransform);
   }

   private void readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations()
   {
      for (int i = 0; i < oneDoFJointPairs.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = oneDoFJointPairs.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         if (pinJoint == null)
            continue;

         revoluteJoint.setQ(pinJoint.getQYoVariable().getDoubleValue());
         revoluteJoint.setQd(pinJoint.getQDYoVariable().getDoubleValue());
         revoluteJoint.setQdd(pinJoint.getQDDYoVariable().getDoubleValue());
         revoluteJoint.setTau(pinJoint.getTau());
      }
   }

   protected void packRootTransform(FloatingRootJointRobot robot, RigidBodyTransform transformToPack)
   {
      robot.getRootJointToWorldTransform(transformToPack);
   }

   @Override
   public long getWallTime()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getMonotonicTime()
   {
      return controllerTimestamp.getLongValue();
   }

   @Override
   public long getSyncTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }

   @Override
   public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
   {
      return jointToSensorOutputMap.get(oneDoFJoint);
   }

   @Override
   public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
   {
      return jointSensorOutputList;
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUOutputs()
   {
      return imuSensors;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutputs()
   {
      return forceSensorDataHolderToUpdate;
   }
}
