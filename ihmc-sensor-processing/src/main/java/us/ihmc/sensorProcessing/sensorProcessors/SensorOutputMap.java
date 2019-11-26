package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.imu.IMUSensor;

/**
 * Simplified sensor output map that just holds all values and does not do any processing
 * 
 * @author jesper
 *
 */
public class SensorOutputMap implements SensorOutputMapReadOnly, RootJointPerfectSensorOutputMapReadOnly
{
   private long wallTime;
   private long monotonicTime;
   private long syncTimestamp;
   private final Map<OneDoFJointBasics, OneDoFJointSensorOutput> jointSensorOutputMap = new HashMap<>();
   private final ArrayList<IMUSensor> imuSensors = new ArrayList<>();
   private final ForceSensorDataHolder forceSensorDataHolder;
   
   /** 
    * Perfect sensors
    */
   private final RigidBodyTransform rootJointTransform = new RigidBodyTransform();
   private final Vector3D rootJointLinearVelocityInBody = new Vector3D();
   private final Vector3D rootJointAngularVelocityInBody = new Vector3D();

   public SensorOutputMap(FullRobotModel fullRobotModel, List<ForceSensorDefinition> forceSensorDefinitions)
   {

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         OneDoFJointSensorOutput jointSensorOutput = new OneDoFJointSensorOutput(joint.getName());
         jointSensorOutput.setPosition(Double.NaN);
         jointSensorOutput.setVelocity(Double.NaN);
         jointSensorOutput.setAcceleration(Double.NaN);
         jointSensorOutput.setEffort(Double.NaN);
         jointSensorOutputMap.put(joint, jointSensorOutput);
      }

      for (IMUDefinition imuDefinition : fullRobotModel.getIMUDefinitions())
      {
         imuSensors.add(new IMUSensor(imuDefinition, null));
      }

      if (forceSensorDefinitions != null)
      {
         this.forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitions);
      }
      else
      {
         this.forceSensorDataHolder = new ForceSensorDataHolder(Collections.emptyList());
      }
   }

   @Override
   public long getWallTime()
   {
      return wallTime;
   }

   public void setWallTime(long wallTime)
   {
      this.wallTime = wallTime;
   }

   public void setMonotonicTime(long monotonicTime)
   {
      this.monotonicTime = monotonicTime;
   }

   @Override
   public long getMonotonicTime()
   {
      return monotonicTime;
   }

   @Override
   public long getSyncTimestamp()
   {
      return syncTimestamp;
   }

   public void setSyncTimestamp(long syncTimestamp)
   {
      this.syncTimestamp = syncTimestamp;
   }

   @Override
   public OneDoFJointSensorOutput getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
   {
      return jointSensorOutputMap.get(oneDoFJoint);
   }

   public void setJointPositionProcessedOutput(OneDoFJointBasics oneDoFJoint, double position)
   {
      getOneDoFJointOutput(oneDoFJoint).setPosition(position);
   }

   public void setJointVelocityProcessedOutput(OneDoFJointBasics oneDoFJoint, double velocity)
   {
      getOneDoFJointOutput(oneDoFJoint).setVelocity(velocity);
   }

   public void setJointAccelerationProcessedOutput(OneDoFJointBasics oneDoFJoint, double acceleration)
   {
      getOneDoFJointOutput(oneDoFJoint).setAcceleration(acceleration);
   }

   public void setJointTauProcessedOutput(OneDoFJointBasics oneDoFJoint, double tau)
   {
      getOneDoFJointOutput(oneDoFJoint).setEffort(tau);
   }

   public void setJointEnabled(OneDoFJointBasics oneDoFJoint, boolean enabled)
   {
      getOneDoFJointOutput(oneDoFJoint).setJointEnabled(enabled);
   }

   @Override
   public List<? extends IMUSensor> getIMUOutputs()
   {
      return imuSensors;
   }

   @Override
   public ForceSensorDataHolder getForceSensorOutputs()
   {
      return forceSensorDataHolder;
   }

   @Override
   public void packRootJointTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(rootJointTransform);
   }
   
   public void setRootJointTransform(RigidBodyTransform transform)
   {
      this.rootJointTransform.set(transform);
   }

   @Override
   public void packRootJointLinearVelocityInBody(Vector3D linearVelocityToPack)
   {
      linearVelocityToPack.set(rootJointLinearVelocityInBody);
   }

   public void setRootJointLinearVelocityInBody(Vector3DReadOnly linearVelocity)
   {
      rootJointLinearVelocityInBody.set(linearVelocity);
   }

   @Override
   public void packRootJointAngularVelocityInBody(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(rootJointAngularVelocityInBody);
   }

   public void setRootJointAngularVelocityInBody(Vector3DReadOnly angularVelocity)
   {
      rootJointAngularVelocityInBody.set(angularVelocity);
   }

}
