package us.ihmc.humanoidRobotics.communication.remote.serialization;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FLAKY)
public class JointConfigurationDataSenderTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final Random random = new Random(5842369L);
   private static final NetworkPorts TCP_PORT = NetworkPorts.createRandomTestPort(random);
   private static final String HOST = "localhost";

	@ContinuousIntegrationTest(estimatedDuration = 8.4)
	@Test(timeout = 42000)
   public void test() throws InterruptedException, IOException
   {
      Random random = new Random(1274L);
      Vector3D[] jointAxes = new Vector3D[] { X, Y, X, Z, X, Y };

      long seedForRobotCreation = 1251235L;

      random.setSeed(seedForRobotCreation);
      RandomFloatingChain testMechanismSendSide = new RandomFloatingChain(random, jointAxes);

      random.setSeed(seedForRobotCreation);
      RandomFloatingChain testMechanismReceiveSide = new RandomFloatingChain(random, jointAxes);

      RandomFloatingChain[] testMechanisms = new RandomFloatingChain[] { testMechanismSendSide, testMechanismReceiveSide };

      NetClassList netClassList = new NetClassList(JointConfigurationData.class);
      netClassList.registerPacketFields(DenseMatrix64F[].class, DenseMatrix64F.class, double[].class);
      final PacketCommunicator server = PacketCommunicator.createTCPPacketCommunicatorServer(TCP_PORT, netClassList);
      final PacketCommunicator client = PacketCommunicator.createTCPPacketCommunicatorClient(HOST, TCP_PORT, netClassList);

      server.connect();
      client.connect();

      JointConfigurationDataSender dataSender = new JointConfigurationDataSender(testMechanismSendSide.getElevator(), server);
      Thread.sleep(100l);

      JointConfigurationDataReceiver dataReceiver = new JointConfigurationDataReceiver(testMechanismReceiveSide.getElevator());
      client.attachListener(JointConfigurationData.class, dataReceiver);
      Thread.sleep(100l);

      int nTests = 5000;
      long maxWaitTimeMillis = 500l;
      for (int i = 0; i < nTests; i++)
      {
         // assert joint data not the same
         for (RandomFloatingChain testMechanism : testMechanisms)
         {
            ScrewTestTools.setRandomPositionAndOrientation(testMechanism.getRootJoint(), random);
            ScrewTestTools.setRandomPositions(testMechanism.getRevoluteJoints(), random);
         }

         assertAllConfigurationsDifferent(testMechanismSendSide, testMechanismReceiveSide, 1e-12);

         // send the data over
         dataSender.send();
         synchronized (dataReceiver)
         {
            dataReceiver.wait(maxWaitTimeMillis);
         }

         // assert joint data the same
         assertAllConfigurationsTheSame(testMechanismSendSide, testMechanismReceiveSide, 1e-16);
      }
      client.close();
      server.close();
   }

   private void assertAllConfigurationsDifferent(RandomFloatingChain testMechanismSendSide, RandomFloatingChain testMechanismReceiveSide, double epsilon)
   {
      assertFalse(isConfigurationEqual(testMechanismSendSide.getRootJoint(), testMechanismReceiveSide.getRootJoint(), epsilon));

      for (int i = 0; i < testMechanismSendSide.getRevoluteJoints().size(); i++)
      {
         assertFalse(isConfigurationEqual(testMechanismSendSide.getRevoluteJoints().get(i), testMechanismReceiveSide.getRevoluteJoints().get(i), epsilon));
      }
   }

   private void assertAllConfigurationsTheSame(RandomFloatingChain testMechanismSendSide, RandomFloatingChain testMechanismReceiveSide, double epsilon)
   {
      assertTrue(isConfigurationEqual(testMechanismSendSide.getRootJoint(), testMechanismReceiveSide.getRootJoint(), epsilon));

      for (int i = 0; i < testMechanismSendSide.getRevoluteJoints().size(); i++)
      {
         RevoluteJoint revoluteJoint1 = testMechanismSendSide.getRevoluteJoints().get(i);
         RevoluteJoint revoluteJoint2 = testMechanismReceiveSide.getRevoluteJoints().get(i);
         assertTrue(isConfigurationEqual(revoluteJoint1, revoluteJoint2, epsilon));
      }
   }

   private boolean isConfigurationEqual(SixDoFJoint sixDoFJoint1, SixDoFJoint sixDoFJoint2, double epsilon)
   {
      RotationMatrix rotation1 = new RotationMatrix();
      sixDoFJoint1.getRotation(rotation1);

      RotationMatrix rotation2 = new RotationMatrix();
      sixDoFJoint2.getRotation(rotation2);
      if (!rotation1.epsilonEquals(rotation2, epsilon))
         return false;

      Vector3D translation1 = new Vector3D();
      sixDoFJoint1.getTranslation(translation1);

      Vector3D translation2 = new Vector3D();
      sixDoFJoint2.getTranslation(translation2);

      if (!translation1.epsilonEquals(translation2, epsilon))
         return false;

      return true;
   }

   private boolean isConfigurationEqual(RevoluteJoint revoluteJoint1, RevoluteJoint revoluteJoint2, double epsilon)
   {
      return Math.abs(revoluteJoint1.getQ() - revoluteJoint2.getQ()) < epsilon;
   }
}
