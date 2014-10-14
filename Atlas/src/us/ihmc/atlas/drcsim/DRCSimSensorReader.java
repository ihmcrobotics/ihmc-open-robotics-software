package us.ihmc.atlas.drcsim;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;
import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DRCSimSensorReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCSimSensorReader");

   private final SocketAddress address = new InetSocketAddress("127.0.0.1", 1234);

   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final SensorProcessing sensorProcessing;
   private final List<OneDoFJoint> jointList;

   private final IMUDefinition imu;

   private final SocketChannel channel;

   private final int jointDataLength;
   private final int imuDataLength;
   private final int forceSensorDataLength;

   public DRCSimSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, DRCRobotSensorInformation sensorInformation,
         SensorFilterParameters sensorFilterParameters, SensorNoiseParameters sensorNoiseParameters, ForceSensorDataHolder forceSensorDataHolderForEstimator,
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap, YoVariableRegistry parentRegistry)
   {
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorFilterParameters, sensorNoiseParameters, registry);

      this.jointList = stateEstimatorSensorDefinitions.getJointPositionSensorDefinitions();
      this.imu = stateEstimatorSensorDefinitions.getAngularVelocitySensorDefinitions().get(0);
      this.forceSensorDataHolderForEstimator = forceSensorDataHolderForEstimator;

      jointDataLength = jointList.size() * 8 * 2;
      imuDataLength = 10 * 8;
      forceSensorDataLength = forceSensorDataHolderForEstimator.getForceSensorDefinitions().size() * 6 * 8;

      try
      {
         channel = SocketChannel.open();
         channel.configureBlocking(true);
         channel.socket().setReceiveBufferSize(jointDataLength + imuDataLength + forceSensorDataLength);
         channel.socket().setKeepAlive(true);
         channel.socket().setReuseAddress(true);
         channel.socket().setSoLinger(false, 0);
         channel.socket().setTcpNoDelay(true);

         System.out.println("[DRCSim] Connecting to " + address);
         channel.connect(address);
         System.out.println("[DRCSim] Connected");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void read()
   {
      ByteBuffer data = ByteBuffer.allocate(jointDataLength + imuDataLength + forceSensorDataLength);
      try
      {
         System.out.println("reading data from channel");
         channel.read(data);
         data.flip();

         System.out.println("Getting timestamp");
         long timestamp = data.getLong();

         System.out.println("Getting joints");
         System.out.println(jointList);
         for (int i = 0; i < jointList.size(); i++)
         {
            OneDoFJoint joint = jointList.get(i);
            sensorProcessing.setJointPositionSensorValue(joint, data.getDouble());
            sensorProcessing.setJointVelocitySensorValue(joint, data.getDouble());
         }

         System.out.println("getting imu data");
         Quat4d orientation = new Quat4d();
         Vector3d linearAcceleration = new Vector3d();
         Vector3d angularVelocity = new Vector3d();

         orientation.setX(data.getDouble());
         orientation.setY(data.getDouble());
         orientation.setZ(data.getDouble());
         orientation.setW(data.getDouble());

         linearAcceleration.setX(data.getDouble());
         linearAcceleration.setY(data.getDouble());
         linearAcceleration.setZ(data.getDouble());

         angularVelocity.setX(data.getDouble());
         angularVelocity.setY(data.getDouble());
         angularVelocity.setZ(data.getDouble());

         sensorProcessing.setOrientationSensorValue(imu, orientation);
         sensorProcessing.setLinearAccelerationSensorValue(imu, linearAcceleration);
         sensorProcessing.setAngularVelocitySensorValue(imu, angularVelocity);

         System.out.println("Getting force sensor data");
         System.out.println(forceSensorDataHolderForEstimator.getForceSensorDefinitions());
         for (int i = 0; i < forceSensorDataHolderForEstimator.getForceSensorDefinitions().size(); i++)
         {
            ForceSensorDefinition definition = forceSensorDataHolderForEstimator.getForceSensorDefinitions().get(i);
            ForceSensorData dataHolder = forceSensorDataHolderForEstimator.get(definition);

            DenseMatrix64F wrench = new DenseMatrix64F(6, 1);

            wrench.set(0, 0, data.getDouble());
            wrench.set(1, 0, data.getDouble());
            wrench.set(2, 0, data.getDouble());
            wrench.set(3, 0, data.getDouble());
            wrench.set(4, 0, data.getDouble());
            wrench.set(5, 0, data.getDouble());
            wrench.set(6, 0, data.getDouble());
            dataHolder.setWrench(wrench);
         }

         sensorProcessing.startComputation(timestamp, timestamp);

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

}
