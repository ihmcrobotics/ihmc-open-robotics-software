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
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMap;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

   private final HashMap<OneDegreeOfFreedomJoint, OneDoFJointBasics> reverseJointLookupMap = new HashMap<>();
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>>();

   private final ArrayList<ImmutablePair<IMUMount, IMUSensor>> imus = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private final TransformReferenceFrame rootJointReferenceFrame = new TransformReferenceFrame("rootJointReferenceFrame", ReferenceFrame.getWorldFrame());

   private final ForceSensorDataHolder forceSensorDataHolderToUpdate;
   private final SensorOutputMap sensorOutputMap;

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   
   private final RotationMatrix imuRotation = new RotationMatrix();
   private final Vector3D imuLinearAcceleration = new Vector3D();
   private final Vector3D imuAngularVelocity = new Vector3D();


   public PerfectSensorIntoSensorOutputMapReader(FloatingRootJointRobot robot, FloatingJointBasics rootJoint, SensorOutputMap sensorOutputMap)
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

   private void createJointRelations(FloatingRootJointRobot robot, FloatingJointBasics rootJoint)
   {
      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            String name = oneDoFJoint.getName();
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robot.getOneDegreeOfFreedomJoint(name);

            ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>(oneDegreeOfFreedomJoint,
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
      sensorOutputMap.setTimestamp(timestamp);
      sensorOutputMap.setControllerTimestamp(timestamp);
      sensorOutputMap.setSensorHeadPPSTimetamp(timestamp);
      
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

   protected ArrayList<ImmutablePair<IMUMount, IMUSensor>> getImus()
   {
      return imus;
   }

   protected void readAndUpdateIMUSensors()
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
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

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
