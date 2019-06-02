package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map.Entry;

import org.apache.commons.lang3.tuple.ImmutablePair;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
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
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SDFPerfectSimulatedSensorReader implements RawSensorReader, SensorOutputMapReadOnly, SensorRawOutputMapReadOnly
{
   private final String name;
   private final FloatingRootJointRobot robot;
   private final FloatingJointBasics rootJoint;
   private final ReferenceFrames referenceFrames;

   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong visionSensorTimestamp = new YoLong("visionSensorTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();

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

            ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>(oneDegreeOfFreedomJoint, oneDoFJoint);
            revoluteJoints.add(jointPair);
         }
      }
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.put(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }

   @Override
   public void initialize()
   {
      read();
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
      this.visionSensorTimestamp.set(timestamp);
      this.sensorHeadPPSTimetamp.set(timestamp);

      if (forceSensorDataHolderToUpdate != null)
      {
         for (Entry<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensorEntry : forceTorqueSensors.entrySet())
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensorEntry.getValue();
            forceTorqueSensor.calculate();
            forceSensorDataHolderToUpdate.setForceSensorValue(forceTorqueSensorEntry.getKey(), forceTorqueSensor.getWrench());
         }
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
      temporaryRootToWorldTransform.normalizeRotationPart();
      rootJoint.setJointConfiguration(temporaryRootToWorldTransform);
   }

   private void readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         if (pinJoint == null) continue;

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
   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getControllerTimestamp()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp.getLongValue();
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQ();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQd();
   }

   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQdd();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getTau();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return new ArrayList<>();
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      return forceSensorDataHolderToUpdate;
   }

   @Override
   public double getJointPositionRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQ();
   }

   @Override
   public double getJointVelocityRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQd();
   }

   @Override
   public double getJointAccelerationRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getQdd();
   }

   @Override
   public double getJointTauRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return oneDoFJoint.getTau();
   }

   @Override
   public boolean isJointEnabled(OneDoFJointBasics oneDoFJoint)
   {
      return true; //oneDoFJoint.isEnabled();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMURawOutputs()
   {
      return new ArrayList<>();
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
   {
      return forceSensorDataHolderToUpdate;
   }

   @Override
   public AtlasAuxiliaryRobotData getAuxiliaryRobotData()
   {
      return null;
   }
}
