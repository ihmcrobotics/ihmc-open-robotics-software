package us.ihmc.gazebo;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.thread.ThreadTools;

public class GazeboSensorReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GazeboSensorReader");

   private final SocketAddress address = new InetSocketAddress("127.0.0.1", 1234);
   private final ByteBuffer data;

   private final LongYoVariable delay = new LongYoVariable("delay", registry);
   private final LongYoVariable timeStampDelta = new LongYoVariable("timeStampDelta", registry);   
   private final SensorProcessing sensorProcessing;
   private final List<OneDoFJoint> jointList;
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions;

   /*
    * Gazebo reports IMU orientations as relative values to their starting value as defined
    * in the model's tree, so we need to store this initial rotation to tack on to all reported rotations.
    */
   private final Quat4d imuToParentLinkRotationOffset;
   private final IMUDefinition imu;

   private SocketChannel channel;

   private final int jointDataLength;
   private final int imuDataLength;
   private final int forceSensorDataLength;
   
   private final Quat4d orientation = new Quat4d();
   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularVelocity = new Vector3d();
   private final DenseMatrix64F wrench = new DenseMatrix64F(6, 1);

   public GazeboSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, DRCRobotSensorInformation sensorInformation,
         StateEstimatorParameters stateEstimatorParameters, RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry parentRegistry)
   {
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, stateEstimatorParameters, registry);

      this.jointList = new ArrayList<>(stateEstimatorSensorDefinitions.getJointSensorDefinitions());
      Collections.sort(jointList, new Comparator<OneDoFJoint>()
      {
         @Override
         public int compare(OneDoFJoint o1, OneDoFJoint o2)
         {
            return o1.getName().compareTo(o2.getName());
         }
      });;
      this.imu = stateEstimatorSensorDefinitions.getIMUSensorDefinitions().get(0);

      imuToParentLinkRotationOffset = new Quat4d();
      imu.getIMUFrame().getTransformToDesiredFrame(imu.getRigidBody().getBodyFixedFrame()).get(imuToParentLinkRotationOffset);

      jointDataLength = jointList.size() * 8 * 2;
      imuDataLength = 10 * 8;
      forceSensorDefinitions = stateEstimatorSensorDefinitions.getForceSensorDefinitions();
      forceSensorDataLength = forceSensorDefinitions.size() * 6 * 8;
      data = ByteBuffer.allocate(16 + jointDataLength + imuDataLength + forceSensorDataLength);
      data.order(ByteOrder.nativeOrder());

      parentRegistry.addChild(registry);
   }

   private long previousTimestamp = 0L;
   private long currentTimestamp = 0L;

   public void connect()
   {
      boolean isConnected = false;
      System.out.println("[GazeboSensorReader] Connecting to " + address);
      
      while(!isConnected)
      {
         try
         {
            channel = SocketChannel.open();
            channel.configureBlocking(true);
            channel.socket().setKeepAlive(true);
            channel.socket().setReuseAddress(true);
            channel.socket().setSoLinger(false, 0);
            channel.socket().setTcpNoDelay(true);

            channel.connect(address);
            isConnected = true;
         }
         catch (IOException e)
         {
            System.out.println("Connect failed.");
            try
            {
               channel.close();
            }
            catch (IOException e1)
            {
               e1.printStackTrace();
            }
            ThreadTools.sleep(3000);
            isConnected = false;
         }
      }

      System.out.println("[GazeboSensorReader] Connected");
   }
   
   @Override
   public void read()
   {
      try
      {
         data.clear();
         while(data.position() < data.limit())
         {
            channel.read(data);
         }
         data.flip();
         
         previousTimestamp = currentTimestamp;
         currentTimestamp = data.getLong();
         timeStampDelta.set(currentTimestamp - previousTimestamp);
         long controlTimestamp = data.getLong();
         
         delay.set(currentTimestamp - controlTimestamp);
         for (int i = 0; i < jointList.size(); i++)
         {
            OneDoFJoint joint = jointList.get(i);
            sensorProcessing.setJointPositionSensorValue(joint, data.getDouble());
            sensorProcessing.setJointVelocitySensorValue(joint, data.getDouble());
         }

         orientation.setW(data.getDouble());
         orientation.setX(data.getDouble());
         orientation.setY(data.getDouble());
         orientation.setZ(data.getDouble());

         orientation.mul(imuToParentLinkRotationOffset, orientation);

         angularVelocity.setX(data.getDouble());
         angularVelocity.setY(data.getDouble());
         angularVelocity.setZ(data.getDouble());

         linearAcceleration.setX(data.getDouble());
         linearAcceleration.setY(data.getDouble());
         linearAcceleration.setZ(data.getDouble());

         sensorProcessing.setOrientationSensorValue(imu, orientation);
         sensorProcessing.setLinearAccelerationSensorValue(imu, linearAcceleration);
         sensorProcessing.setAngularVelocitySensorValue(imu, angularVelocity);

         for (int i = 0; i < forceSensorDefinitions.size(); i++)
         {
            ForceSensorDefinition definition = forceSensorDefinitions.get(i);

            wrench.set(0, 0, data.getDouble());
            wrench.set(1, 0, data.getDouble());
            wrench.set(2, 0, data.getDouble());
            wrench.set(3, 0, data.getDouble());
            wrench.set(4, 0, data.getDouble());
            wrench.set(5, 0, data.getDouble());
            sensorProcessing.setForceSensorValue(definition, wrench);
         }

         sensorProcessing.startComputation(currentTimestamp, currentTimestamp, -1);

      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }
   
   @Override public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
