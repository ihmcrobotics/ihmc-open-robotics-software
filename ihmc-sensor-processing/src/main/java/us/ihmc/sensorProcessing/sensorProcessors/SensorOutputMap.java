package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
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
   private long timestamp;
   private long visionSensorTimestamp;
   private long sensorHeadPPSTimestamp;
   private final TObjectDoubleMap<OneDoFJoint> jointPosition = new TObjectDoubleHashMap<>();
   private final TObjectDoubleMap<OneDoFJoint> jointVelocity = new TObjectDoubleHashMap<>();
   private final TObjectDoubleMap<OneDoFJoint> jointAcceleration = new TObjectDoubleHashMap<>();
   private final TObjectDoubleMap<OneDoFJoint> jointTau = new TObjectDoubleHashMap<>();
   private final TObjectIntMap<OneDoFJoint> jointEnabled = new TObjectIntHashMap<OneDoFJoint>();
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

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         jointPosition.put(joint, Double.NaN);
         jointVelocity.put(joint, Double.NaN);
         jointAcceleration.put(joint, Double.NaN);
         jointTau.put(joint, Double.NaN);
         jointEnabled.put(joint, 0);
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
   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp;
   }

   public void setVisionSensorTimestamp(long visionSensorTimestamp)
   {
      this.visionSensorTimestamp = visionSensorTimestamp;
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimestamp;
   }

   public void setSensorHeadPPSTimetamp(long sensorHeadPPSTimestamp)
   {
      this.sensorHeadPPSTimestamp = sensorHeadPPSTimestamp;
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointPosition.get(oneDoFJoint);
   }

   public void setJointPositionProcessedOutput(OneDoFJoint oneDoFJoint, double position)
   {
      this.jointPosition.put(oneDoFJoint, position);
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointVelocity.get(oneDoFJoint);
   }

   public void setJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint, double velocity)
   {
      this.jointVelocity.put(oneDoFJoint, velocity);
   }

   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointAcceleration.get(oneDoFJoint);
   }

   public void setJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint, double acceleration)
   {
      this.jointAcceleration.put(oneDoFJoint, acceleration);
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointTau.get(oneDoFJoint);
   }

   public void setJointTauProcessedOutput(OneDoFJoint oneDoFJoint, double tau)
   {
      this.jointTau.put(oneDoFJoint, tau);
   }

   @Override
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      return jointEnabled.get(oneDoFJoint) != 0;
   }

   public void setJointEnabled(OneDoFJoint oneDoFJoint, boolean enabled)
   {
      this.jointEnabled.put(oneDoFJoint, enabled ? 1 : 0);
   }

   @Override
   public List<? extends IMUSensor> getIMUProcessedOutputs()
   {
      return imuSensors;
   }

   @Override
   public ForceSensorDataHolder getForceSensorProcessedOutputs()
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
