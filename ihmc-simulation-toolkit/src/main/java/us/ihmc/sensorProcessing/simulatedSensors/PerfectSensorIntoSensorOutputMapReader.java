package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMap;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Perfect sensor reader that reads data from a simulated robot and outputs data in a SensorOutputMap
 * 
 * This class does not adjust the values.
 * 
 * @author jesper
 *
 */
public class PerfectSensorIntoSensorOutputMapReader implements RawSensorReader
{
   private final String name;
   private final FloatingRootJointRobot robot;

   private final HashMap<OneDegreeOfFreedomJoint, OneDoFJoint> reverseJointLookupMap = new HashMap<>();
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();

   private final ArrayList<ImmutablePair<IMUMount, IMUSensor>> imus = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong visionSensorTimestamp = new YoLong("visionSensorTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private final TransformReferenceFrame rootJointReferenceFrame = new TransformReferenceFrame("rootJointReferenceFrame", ReferenceFrame.getWorldFrame());

   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private final SensorOutputMap sensorOutputMap;

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   
   private final RotationMatrix imuRotation = new RotationMatrix();
   private final Vector3D imuLinearAcceleration = new Vector3D();
   private final Vector3D imuAngularVelocity = new Vector3D();


   public PerfectSensorIntoSensorOutputMapReader(FloatingRootJointRobot robot, FloatingInverseDynamicsJoint rootJoint, SensorOutputMap sensorOutputMap)
   {
      name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.forceSensorDataHolderToUpdate = sensorOutputMap.getForceSensorProcessedOutputs();
      this.sensorOutputMap = sensorOutputMap;

      createJointRelations(robot, rootJoint);


      HashMap<String, IMUSensor> imuSensors = new HashMap<>();
      for (IMUSensor sensor : sensorOutputMap.getIMUProcessedOutputs())
      {
         imuSensors.put(sensor.getSensorName(), sensor);
      }

      ArrayList<IMUMount> imuMounts = new ArrayList<>();
      robot.getIMUMounts(imuMounts);

      for (IMUMount imuMount : imuMounts)
      {
         IMUSensor imuDefinition = imuSensors.get(imuMount.getName());
         if (imuDefinition == null)
         {
            throw new RuntimeException("IMU Definition not found in robot model");
         }
         imus.add(new ImmutablePair<IMUMount, IMUSensor>(imuMount, imuDefinition));
      }

   }

   private void createJointRelations(FloatingRootJointRobot robot, FloatingInverseDynamicsJoint rootJoint)
   {
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor());

      for (InverseDynamicsJoint joint : jointsArray)
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            String name = oneDoFJoint.getName();
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robot.getOneDegreeOfFreedomJoint(name);

            ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDegreeOfFreedomJoint,
                                                                                                                                    oneDoFJoint);
            revoluteJoints.add(jointPair);
            reverseJointLookupMap.put(oneDegreeOfFreedomJoint, oneDoFJoint);
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
      readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations();
      readAndUpdateRootJointPositionAndOrientation();
      readAndUpdateRootJointAngularAndLinearVelocity();
      readAndUpdateIMUSensors();

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

   private void readAndUpdateIMUSensors()
   {
      for(int i = 0; i < imus.size(); i++)
      {
         ImmutablePair<IMUMount, IMUSensor> pair = imus.get(i);
         
         IMUMount mount = pair.getLeft();
         IMUSensor sensor = pair.getRight();

         mount.getLinearAccelerationInBody(imuLinearAcceleration);
         mount.getAngularVelocityInBody(imuAngularVelocity);
         mount.getOrientation(imuRotation);
         
         sensor.setLinearAccelerationMeasurement(imuLinearAcceleration);
         sensor.setAngularVelocityMeasurement(imuAngularVelocity);
         sensor.setOrientationMeasurement(imuRotation);
         
      }
   }

   private void readAndUpdateRootJointAngularAndLinearVelocity()
   {

      linearVelocity.setToZero(ReferenceFrame.getWorldFrame());
      robot.getVelocityInWorld(linearVelocity);
      linearVelocity.changeFrame(rootJointReferenceFrame);

      angularVelocity.setToZero(rootJointReferenceFrame);
      robot.getAngularVelocityInBody(angularVelocity);

      sensorOutputMap.setRootJointLinearVelocityInBody(linearVelocity);
      sensorOutputMap.setRootJointAngularVelocityInBody(angularVelocity);
   }

   private void readAndUpdateRootJointPositionAndOrientation()
   {
      packRootTransform(robot, temporaryRootToWorldTransform);
      temporaryRootToWorldTransform.normalizeRotationPart();
      sensorOutputMap.setRootJointTransform(temporaryRootToWorldTransform);
      rootJointReferenceFrame.setTransformAndUpdate(temporaryRootToWorldTransform);
   }

   private void readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         if (pinJoint == null)
            continue;

         sensorOutputMap.setJointPositionProcessedOutput(revoluteJoint, pinJoint.getQ());
         sensorOutputMap.setJointVelocityProcessedOutput(revoluteJoint, pinJoint.getQD());
         sensorOutputMap.setJointAccelerationProcessedOutput(revoluteJoint, pinJoint.getQDD());
         sensorOutputMap.setJointEnabled(revoluteJoint, pinJoint.isDynamic());

      }
   }

   protected void packRootTransform(FloatingRootJointRobot robot, RigidBodyTransform transformToPack)
   {
      robot.getRootJointToWorldTransform(transformToPack);
   }

}
