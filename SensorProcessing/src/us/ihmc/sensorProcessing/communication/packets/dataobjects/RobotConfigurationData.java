package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.ArrayList;
import java.util.Random;
import java.util.zip.CRC32;

import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class RobotConfigurationData extends Packet<RobotConfigurationData>
{
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
   public RobotMotionStatus robotMotionStatus;
   public AuxiliaryRobotData auxiliaryRobotData;

   public int lastReceivedPacketTypeID;
   public long lastReceivedPacketUniqueId;
   public long lastReceivedPacketRobotTimestamp;

   public RobotConfigurationData(Random random)
   {
      timestamp = random.nextLong();
      sensorHeadPPSTimestamp = random.nextLong();
      jointNameHash = random.nextInt(10000);

      int size = Math.abs(random.nextInt(1000));

      jointAngles = new float[size];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = random.nextFloat();
      }

      jointVelocities = new float[size];
      for (int i = 0; i < jointVelocities.length; i++)
      {
         jointVelocities[i] = random.nextFloat();
      }

      jointTorques = new float[size];
      for (int i = 0; i < jointTorques.length; i++)
      {
         jointTorques[i] = random.nextFloat();
      }

      rootTranslation = RandomTools.generateRandomVector3f(random);
      rootOrientation = RandomTools.generateRandomQuaternion4f(random);

      size = Math.abs(random.nextInt(1000));
      momentAndForceDataAllForceSensors = new float[size][Wrench.SIZE];
      for (int i = 0; i < momentAndForceDataAllForceSensors.length; i++)
      {
         for (int j = 0; j < Wrench.SIZE; j++)
         {
            momentAndForceDataAllForceSensors[i][j] = random.nextFloat();
         }
      }
   }

   public RobotConfigurationData()
   {
      // empty constructor for serialization
   }

   public RobotConfigurationData(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, AuxiliaryRobotData auxiliaryRobotData,
         IMUDefinition[] imuDefinitions)
   {
      jointAngles = new float[joints.length];
      jointVelocities = new float[joints.length];
      jointTorques = new float[joints.length];
      momentAndForceDataAllForceSensors = new float[forceSensorDefinitions.length][Wrench.SIZE];

      imuSensorData = new IMUPacket[imuDefinitions.length];
      for (int sensorNumber = 0; sensorNumber < imuSensorData.length; sensorNumber++)
      {
         imuSensorData[sensorNumber] = new IMUPacket();
      }

      jointNameHash = calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);
      this.auxiliaryRobotData = auxiliaryRobotData;
   }

   public void setJointState(ArrayList<OneDoFJoint> newJointData)
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

   public void setRootTranslation(Vector3D rootTranslation)
   {
      this.rootTranslation.set(rootTranslation);
   }

   public void setRootOrientation(Quaternion rootOrientation)
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

   public void setRobotMotionStatus(RobotMotionStatus robotMotionStatus)
   {
      this.robotMotionStatus = robotMotionStatus;
   }

   public RobotMotionStatus getRobotMotionStatus()
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

   public AuxiliaryRobotData getAuxiliaryRobotData()
   {
      return auxiliaryRobotData;
   }

   public void setAuxiliaryRobotData(AuxiliaryRobotData auxiliaryRobotData)
   {
      if (this.auxiliaryRobotData != null && auxiliaryRobotData != null)
      {
         this.auxiliaryRobotData.setAuxiliaryRobotData(auxiliaryRobotData);
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
