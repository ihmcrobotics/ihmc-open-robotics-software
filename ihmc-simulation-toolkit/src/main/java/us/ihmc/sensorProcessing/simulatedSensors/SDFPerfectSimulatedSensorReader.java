package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunction;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
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
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SDFPerfectSimulatedSensorReader implements RawSensorReader, SensorOutputMapReadOnly
{
   private final String name;
   private final FloatingRootJointRobot robot;
   private final FloatingJointBasics rootJoint;
   private final ReferenceFrames referenceFrames;

   private final List<Runnable> oneDoFJointStateUpdaters = new ArrayList<>();
   private final List<OneDoFJointStateReadOnly> jointSensorOutputList = new ArrayList<>();
   private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointToSensorOutputMap = new HashMap<>();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<>();
   private final PairList<IMUSensor, IMUMount> imuSensorPairs = new PairList<>();
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

            if (oneDoFJoint instanceof InvertedFourBarJoint)
            {
               String jointAName = ((InvertedFourBarJoint) oneDoFJoint).getJointA().getName();
               String jointBName = ((InvertedFourBarJoint) oneDoFJoint).getJointB().getName();
               String jointCName = ((InvertedFourBarJoint) oneDoFJoint).getJointC().getName();
               String jointDName = ((InvertedFourBarJoint) oneDoFJoint).getJointD().getName();
               oneDoFJointStateUpdaters.add(new InvertedFourBarJointStateUpdater((InvertedFourBarJoint) oneDoFJoint,
                                                                                 robot.getOneDegreeOfFreedomJoint(jointAName),
                                                                                 robot.getOneDegreeOfFreedomJoint(jointBName),
                                                                                 robot.getOneDegreeOfFreedomJoint(jointCName),
                                                                                 robot.getOneDegreeOfFreedomJoint(jointDName)));
            }
            else
            {
               oneDoFJointStateUpdaters.add(new OneDoFJointStateUpdater(oneDoFJoint, robot.getOneDegreeOfFreedomJoint(oneDoFJoint.getName())));
            }

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
      addIMUSensor(imuDefinition, null);
   }

   public void addIMUSensor(IMUDefinition imuDefinition, IMUMount imuMount)
   {
      IMUSensor imuSensor = new IMUSensor(imuDefinition, null);
      imuSensorPairs.add(new ImmutablePair<>(imuSensor, imuMount));
      imuSensors.add(imuSensor);
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
   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Quaternion orientation = new Quaternion();

   @Override
   public void read()
   {
      // Think about adding root body acceleration to the fullrobotmodel
      for (int i = 0; i < oneDoFJointStateUpdaters.size(); i++)
      {
         oneDoFJointStateUpdaters.get(i).run();
      }
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

      for (ImmutablePair<IMUSensor, IMUMount> imuSensorPair : imuSensorPairs)
      {
         IMUSensor imuSensor = imuSensorPair.getLeft();
         IMUMount imuMount = imuSensorPair.getRight();

         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();

         if (imuMount != null)
         {
            imuMount.getLinearAccelerationInBody(linearAcceleration);
            imuMount.getAngularVelocityInBody(angularVelocity);
            imuMount.getOrientation(orientation);

            imuSensor.setLinearAccelerationMeasurement(linearAcceleration);
            imuSensor.setAngularVelocityMeasurement(angularVelocity);
            imuSensor.setOrientationMeasurement(orientation);
         }
         else
         {
            Twist twist = new Twist(imuSensor.getMeasurementLink().getBodyFixedFrame().getTwistOfFrame());
            twist.changeFrame(measurementFrame);
            imuSensor.setAngularVelocityMeasurement(twist.getAngularPart());
            imuSensor.setOrientationMeasurement(measurementFrame.getTransformToRoot().getRotation());

            // TODO add acceleration
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
      temporaryRootToWorldTransform.getRotation().normalize();
      rootJoint.setJointConfiguration(temporaryRootToWorldTransform);
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

   public static class OneDoFJointStateUpdater implements Runnable
   {
      private final OneDoFJointBasics inverseDynamicsJoint;
      private final OneDegreeOfFreedomJoint scsJoint;

      public OneDoFJointStateUpdater(OneDoFJointBasics inverseDynamicsJoint, OneDegreeOfFreedomJoint scsJoint)
      {
         this.inverseDynamicsJoint = inverseDynamicsJoint;
         this.scsJoint = scsJoint;
      }

      public void run()
      {
         if (scsJoint == null)
            return;

         inverseDynamicsJoint.setQ(scsJoint.getQ());
         inverseDynamicsJoint.setQd(scsJoint.getQD());
         inverseDynamicsJoint.setQdd(scsJoint.getQDD());
         inverseDynamicsJoint.setTau(scsJoint.getTau());
      }
   }

   public static class InvertedFourBarJointStateUpdater implements Runnable
   {
      private final InvertedFourBarJoint invertedFourBarJoint;
      private final OneDegreeOfFreedomJoint scsJointA, scsJointB, scsJointC, scsJointD;

      public InvertedFourBarJointStateUpdater(InvertedFourBarJoint invertedFourBarJoint, OneDegreeOfFreedomJoint scsJointA, OneDegreeOfFreedomJoint scsJointB,
                                              OneDegreeOfFreedomJoint scsJointC, OneDegreeOfFreedomJoint scsJointD)
      {
         this.invertedFourBarJoint = invertedFourBarJoint;
         this.scsJointA = scsJointA;
         this.scsJointB = scsJointB;
         this.scsJointC = scsJointC;
         this.scsJointD = scsJointD;
      }

      @Override
      public void run()
      {
         FourBarKinematicLoopFunction fourBarFunction = invertedFourBarJoint.getFourBarFunction();

         DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
         if (scsJointA != null && scsJointD != null)
         {
            invertedFourBarJoint.setQ(scsJointA.getQ() + scsJointD.getQ());
            invertedFourBarJoint.setQd(scsJointA.getQD() + scsJointD.getQD());
            invertedFourBarJoint.setQdd(scsJointA.getQDD() + scsJointD.getQDD());
            fourBarFunction.updateState(false, false);
            invertedFourBarJoint.setTau(loopJacobian.get(0) * scsJointA.getTau() + loopJacobian.get(3) * scsJointD.getTau());
         }
         else
         {
            invertedFourBarJoint.setQ(scsJointB.getQ() + scsJointC.getQ());
            invertedFourBarJoint.setQd(scsJointB.getQD() + scsJointC.getQD());
            invertedFourBarJoint.setQdd(scsJointB.getQDD() + scsJointC.getQDD());
            fourBarFunction.updateState(false, false);
            invertedFourBarJoint.setTau(loopJacobian.get(1) * scsJointB.getTau() + loopJacobian.get(2) * scsJointC.getTau());
         }
      }
   }
}
