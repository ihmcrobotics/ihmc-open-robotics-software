package us.ihmc.atlas.packets;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.ByteArrayOutputStream;

import org.junit.jupiter.api.Test;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Output;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class AtlasRobotConfigurationDataTest
{

   /**
    * This test tests if the serialized RobotConfigurationData fits completely in a TCP frame or UDP packet, 
    * in other words has a size less that 1460 bytes.
    * 
    * This allows sending data without fragmentation, reducing jitter and making UDP communication simpler.
    */
   @Test
   public void testSerializedSize()
   {
      ByteArrayOutputStream os = new ByteArrayOutputStream(1500);
      Output output = new Output(os);
      Kryo kryo = new Kryo();
      new IHMCCommunicationKryoNetClassList().registerWithKryo(kryo);

      OneDoFJointBasics[] joints = new OneDoFJointBasics[31];
      RigidBodyBasics body = new RigidBody("aap", ReferenceFrame.getWorldFrame());
      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = new RevoluteJoint("noot", body, new RigidBodyTransform(), new Vector3D(1, 0, 0));
      }
      
      RigidBodyBasics body2 = new RigidBody("mies", joints[0], new Matrix3D(), 0.0, new RigidBodyTransform());
      IMUDefinition imuSensorDefinitions[] = new IMUDefinition[3];
      for (int i = 0; i < imuSensorDefinitions.length; i++)
      {
         imuSensorDefinitions[i] = new IMUDefinition("fake_imu" + i, body2, new RigidBodyTransform());
      }
      
      ForceSensorDefinition forceSensorDefinitions[] = new ForceSensorDefinition[4];
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame("wim", body2, new RigidBodyTransform());
         forceSensorDefinitions[i] = new ForceSensorDefinition("wim", body2, sensorFrame);
      }

      RobotConfigurationData data = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuSensorDefinitions);
      kryo.writeClassAndObject(output, data);
      output.flush();

      int length = os.toByteArray().length;
      assertTrue("RobotConfigurationData is to large " + length, length < 1460);
   }
}
