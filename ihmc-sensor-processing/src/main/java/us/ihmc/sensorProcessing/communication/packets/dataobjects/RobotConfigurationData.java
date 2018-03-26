package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.List;
import java.util.zip.CRC32;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SpatialVectorMessage;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class RobotConfigurationData extends Packet<RobotConfigurationData>
{
   public static final byte ROBOT_MOTION_STATUS_STANDING = 0;
   public static final byte ROBOT_MOTION_STATUS_IN_MOTION = 1;

   public long timestamp = 0;
   public long sensorHeadPPSTimestamp;
   public int jointNameHash;
   public TFloatArrayList jointAngles = new TFloatArrayList();
   public TFloatArrayList jointVelocities = new TFloatArrayList();
   public TFloatArrayList jointTorques = new TFloatArrayList();

   public Vector3D32 rootTranslation = new Vector3D32();
   public Vector3D32 pelvisLinearVelocity = new Vector3D32();
   public Vector3D32 pelvisAngularVelocity = new Vector3D32();
   public Quaternion32 rootOrientation = new Quaternion32();

   public Vector3D32 pelvisLinearAcceleration = new Vector3D32();
   //   public DenseMatrix64F[] momentAndForceDataAllForceSensors;
   public RecyclingArrayListPubSub<SpatialVectorMessage> momentAndForceDataAllForceSensors = new RecyclingArrayListPubSub<>(SpatialVectorMessage.class, SpatialVectorMessage::new, 2);
   public RecyclingArrayListPubSub<IMUPacket> imuSensorData = new RecyclingArrayListPubSub<>(IMUPacket.class, IMUPacket::new, 1);
   public byte robotMotionStatus;

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
      MessageTools.copyData(other.jointAngles, jointAngles);
      MessageTools.copyData(other.jointVelocities, jointVelocities);
      MessageTools.copyData(other.jointTorques, jointTorques);

      rootTranslation.set(other.rootTranslation);
      rootOrientation.set(other.rootOrientation);
      pelvisLinearVelocity.set(other.pelvisLinearVelocity);
      pelvisAngularVelocity.set(other.pelvisAngularVelocity);
      pelvisLinearAcceleration.set(other.pelvisLinearAcceleration);

      MessageTools.copyData(other.momentAndForceDataAllForceSensors, momentAndForceDataAllForceSensors);
      MessageTools.copyData(other.imuSensorData, imuSensorData);
      robotMotionStatus = other.robotMotionStatus;
      lastReceivedPacketTypeID = other.lastReceivedPacketTypeID;
      lastReceivedPacketUniqueId = other.lastReceivedPacketUniqueId;
      lastReceivedPacketRobotTimestamp = other.lastReceivedPacketRobotTimestamp;
      setPacketInformation(other);
   }

   public void setJointState(OneDoFJoint[] newJointData)
   {
      jointAngles.reset();
      jointVelocities.reset();
      jointTorques.reset();

      for (int i = 0; i < newJointData.length; i++)
      {
         jointAngles.add((float) newJointData[i].getQ());
         jointVelocities.add((float) newJointData[i].getQd());
         jointTorques.add((float) newJointData[i].getTauMeasured());
      }
   }
   
   public void setJointState(List<? extends OneDoFJoint> newJointData)
   {
      jointAngles.reset();
      jointVelocities.reset();
      jointTorques.reset();

      for (int i = 0; i < newJointData.size(); i++)
      {
         jointAngles.add((float) newJointData.get(i).getQ());
         jointVelocities.add((float) newJointData.get(i).getQd());
         jointTorques.add((float) newJointData.get(i).getTauMeasured());
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

   public TFloatArrayList getJointAngles()
   {
      return jointAngles;
   }

   public TFloatArrayList getJointVelocities()
   {
      return jointVelocities;
   }

   public TFloatArrayList getJointTorques()
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

      for (int i = 0; i < jointAngles.size(); i++)
      {
         if (Math.abs(jointAngles.get(i) - other.jointAngles.get(i)) > epsilon)
         {
            System.out.println(i);
            System.out.println("Diff: " + Math.abs(jointAngles.get(i) - other.jointAngles.get(i)) + ", this: " + jointAngles.get(i) + ", other: " + other.jointAngles.get(i));
            return false;
         }
      }

      return timestamp == other.timestamp;
   }

   public SpatialVectorMessage getMomentAndForceVectorForSensor(int sensorNumber)
   {
      return momentAndForceDataAllForceSensors.get(sensorNumber);
   }

   public IMUPacket getImuPacketForSensor(int sensorNumber)
   {
      return imuSensorData.get(sensorNumber);
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
