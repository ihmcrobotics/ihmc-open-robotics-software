package us.ihmc.atlas.packets;

import static org.junit.Assert.assertTrue;

import java.io.ByteArrayOutputStream;

import javax.vecmath.Matrix3d;

import org.junit.Test;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AtlasRobotConfigurationDataTest
{

   /**
    * This test tests if the serialized RobotConfigurationData fits completely in a TCP frame or UDP packet, 
    * in other words has a size less that 1460 bytes.
    * 
    * This allows sending data without fragmentation, reducing jitter and making UDP communication simpler.
    */
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSerializedSize()
   {
      ByteArrayOutputStream os = new ByteArrayOutputStream(1500);
      Output output = new Output(os);
      Kryo kryo = new Kryo();
      new IHMCCommunicationKryoNetClassList().registerWithKryo(kryo);

      OneDoFJoint[] joints = new OneDoFJoint[31];
      RigidBody body = new RigidBody("aap", ReferenceFrame.getWorldFrame());
      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = new RevoluteJoint("noot", body, ReferenceFrame.getWorldFrame(), new FrameVector(ReferenceFrame.getWorldFrame(), 1, 0, 0));
      }
      
      RigidBody body2 = new RigidBody("mies", new RigidBodyInertia(ReferenceFrame.getWorldFrame(), new Matrix3d(), 10.0), joints[0]);
      IMUDefinition imuSensorDefinitions[] = new IMUDefinition[3];
      for (int i = 0; i < imuSensorDefinitions.length; i++)
      {
         imuSensorDefinitions[i] = new IMUDefinition("fake_imu" + i, body2, new RigidBodyTransform());
      }
      
      ForceSensorDefinition forceSensorDefinitions[] = new ForceSensorDefinition[4];
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         forceSensorDefinitions[i] = new ForceSensorDefinition("wim", body2, new RigidBodyTransform());
      }

      AtlasAuxiliaryRobotData auxiliaryRobotData = new AtlasAuxiliaryRobotData();

      RobotConfigurationData data = new RobotConfigurationData(joints, forceSensorDefinitions, auxiliaryRobotData, imuSensorDefinitions);
      kryo.writeClassAndObject(output, data);
      output.flush();

      int length = os.toByteArray().length;
      assertTrue("RobotConfigurationData is to large " + length, length < 1460);
   }
}
