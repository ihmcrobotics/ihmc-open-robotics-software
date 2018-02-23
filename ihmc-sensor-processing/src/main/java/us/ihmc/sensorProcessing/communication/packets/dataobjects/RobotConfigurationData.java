package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.Arrays;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class RobotConfigurationData extends Packet<RobotConfigurationData>
{
   public static final byte ROBOT_MOTION_STATUS_STANDING = 0;
   public static final byte ROBOT_MOTION_STATUS_IN_MOTION = 1;

   public long timestamp = 0;
   public long sensorHeadPPSTimestamp;
   public int jointNameHash;
   public float[] jointAngles;
   public float[] jointVelocities;
   public float[] jointTorques;

   public Vector3D32 rootTranslation = new Vector3D32();
   public Vector3D32 pelvisLinearVelocity = new Vector3D32();
   public Vector3D32 pelvisAngularVelocity = new Vector3D32();
   public Quaternion32 rootOrientation = new Quaternion32();

   public Vector3D32 pelvisLinearAcceleration = new Vector3D32();
   //   public DenseMatrix64F[] momentAndForceDataAllForceSensors;
   public float[][] momentAndForceDataAllForceSensors;
   public IMUPacket[] imuSensorData;
   public byte robotMotionStatus;
   public AtlasAuxiliaryRobotData auxiliaryRobotData;

   public int lastReceivedPacketTypeID;
   public long lastReceivedPacketUniqueId;
   public long lastReceivedPacketRobotTimestamp;

   public RobotConfigurationData()
   {
      // empty constructor for serialization
   }

   @Override
   public void set(RobotConfigurationData other)
   {
      timestamp = other.timestamp;
      sensorHeadPPSTimestamp = other.sensorHeadPPSTimestamp;
      jointNameHash = other.jointNameHash;
      jointAngles = Arrays.copyOf(other.jointAngles, other.jointAngles.length);
      jointVelocities = Arrays.copyOf(other.jointVelocities, other.jointVelocities.length);
      jointTorques = Arrays.copyOf(other.jointTorques, other.jointTorques.length);

      rootTranslation.set(other.rootTranslation);
      rootOrientation.set(other.rootOrientation);
      pelvisLinearVelocity.set(other.pelvisLinearVelocity);
      pelvisAngularVelocity.set(other.pelvisAngularVelocity);
      pelvisLinearAcceleration.set(other.pelvisLinearAcceleration);
      momentAndForceDataAllForceSensors = new float[other.momentAndForceDataAllForceSensors.length][Wrench.SIZE];
      for (int i = 0; i < momentAndForceDataAllForceSensors.length; i++)
         momentAndForceDataAllForceSensors[i] = Arrays.copyOf(other.momentAndForceDataAllForceSensors[i], Wrench.SIZE);
      imuSensorData = new IMUPacket[other.imuSensorData.length];
      for (int i = 0; i < imuSensorData.length; i++)
      {
         imuSensorData[i] = new IMUPacket();
         imuSensorData[i].set(other.imuSensorData[i]);
      }
      robotMotionStatus = other.robotMotionStatus;
      auxiliaryRobotData.set(other.auxiliaryRobotData);
      lastReceivedPacketTypeID = other.lastReceivedPacketTypeID;
      lastReceivedPacketUniqueId = other.lastReceivedPacketUniqueId;
      lastReceivedPacketRobotTimestamp = other.lastReceivedPacketRobotTimestamp;
      setPacketInformation(other);
   }

   public void setJointState(OneDoFJoint[] newJointData)
   {
      if (newJointData.length != jointAngles.length)
         throw new RuntimeException("Array size does not match");

      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = (float) newJointData[i].getQ();
         jointVelocities[i] = (float) newJointData[i].getQd();
         jointTorques[i] = (float) newJointData[i].getTauMeasured();
      }
   }
   
   public void setJointState(List<? extends OneDoFJoint> newJointData)
   {
      if (newJointData.size() != jointAngles.length)
         throw new RuntimeException("Array size does not match");

      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = (float) newJointData.get(i).getQ();
         jointVelocities[i] = (float) newJointData.get(i).getQd();
         jointTorques[i] = (float) newJointData.get(i).getTauMeasured();
      }
   }

   public void setRootTranslation(Tuple3DReadOnly rootTranslation)
   {
      this.rootTranslation.set(rootTranslation);
   }

   public void setRootOrientation(QuaternionReadOnly rootOrientation)
   {
      this.rootOrientation.set(rootOrientation);
   }

   public float[] getJointAngles()
   {
      return jointAngles;
   }

   public float[] getJointVelocities()
   {
      return jointVelocities;
   }

   public float[] getJointTorques()
   {
      return jointTorques;
   }

   public Vector3D32 getPelvisTranslation()
   {
      return rootTranslation;
   }

   public Quaternion32 getPelvisOrientation()
   {
      return rootOrientation;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public boolean epsilonEquals(RobotConfigurationData other, double epsilon)
   {
      if (!rootTranslation.epsilonEquals(other.rootTranslation, 1e-3f))
      {
         return false;
      }

      if (!RotationTools.quaternionEpsilonEquals(rootOrientation, other.rootOrientation, 1e-3f))
      {
         return false;
      }

      for (int i = 0; i < jointAngles.length; i++)
      {
         if (Math.abs(jointAngles[i] - other.jointAngles[i]) > epsilon)
         {
            System.out.println(i);
            System.out.println("Diff: " + Math.abs(jointAngles[i] - other.jointAngles[i]) + ", this: " + jointAngles[i] + ", other: " + other.jointAngles[i]);
            return false;
         }
      }

      return timestamp == other.timestamp;
   }

   public float[] getMomentAndForceVectorForSensor(int sensorNumber)
   {
      return momentAndForceDataAllForceSensors[sensorNumber];
   }

   public IMUPacket getImuPacketForSensor(int sensorNumber)
   {
      return imuSensorData[sensorNumber];
   }

   public static int calculateJointNameHash(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJoint joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

   public void setSensorHeadPPSTimestamp(long sensorHeadPPSTimestamp)
   {
      this.sensorHeadPPSTimestamp = sensorHeadPPSTimestamp;
   }

   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimestamp;
   }

   public void setRobotMotionStatus(byte robotMotionStatus)
   {
      this.robotMotionStatus = robotMotionStatus;
   }

   public byte getRobotMotionStatus()
   {
      return robotMotionStatus;
   }

   public Vector3D32 getPelvisLinearVelocity()
   {
      return pelvisLinearVelocity;
   }

   public Vector3D32 getPelvisAngularVelocity()
   {
      return pelvisAngularVelocity;
   }

   public void setPelvisLinearVelocity(Vector3D pelvisLinearVelocityToPack)
   {
      pelvisLinearVelocity.set(pelvisLinearVelocityToPack);
   }

   public void setPelvisAngularVelocity(Vector3D pelvisAngularVelocityToPack)
   {
      pelvisAngularVelocity.set(pelvisAngularVelocityToPack);
   }

   public AtlasAuxiliaryRobotData getAuxiliaryRobotData()
   {
      return auxiliaryRobotData;
   }

   public void setAuxiliaryRobotData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      if (this.auxiliaryRobotData != null && auxiliaryRobotData != null)
      {
         this.auxiliaryRobotData.set(auxiliaryRobotData);
      }
   }

   public Vector3D32 getPelvisLinearAcceleration()
   {
      return pelvisLinearAcceleration;
   }

   public void setPelvisLinearAcceleration(Vector3D pelvisLinearAcceleration)
   {
      this.pelvisLinearAcceleration.set(pelvisLinearAcceleration);
   }

   public int getLastReceivedPacketTypeID()
   {
      return lastReceivedPacketTypeID;
   }

   public void setLastReceivedPacketTypeID(int lastReceivedPacketTypeID)
   {
      this.lastReceivedPacketTypeID = lastReceivedPacketTypeID;
   }

   public long getLastReceivedPacketUniqueId()
   {
      return lastReceivedPacketUniqueId;
   }

   public void setLastReceivedPacketUniqueId(long lastReceivedPacketUniqueId)
   {
      this.lastReceivedPacketUniqueId = lastReceivedPacketUniqueId;
   }

   public long getLastReceivedPacketRobotTimestamp()
   {
      return lastReceivedPacketRobotTimestamp;
   }

   public void setLastReceivedPacketRobotTimestamp(long lastReceivedPacketRobotTimestamp)
   {
      this.lastReceivedPacketRobotTimestamp = lastReceivedPacketRobotTimestamp;
   }

}
