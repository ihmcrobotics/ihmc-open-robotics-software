package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;

public class JointWrenchSensorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testStaticallyHangingMasses() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3d sensorOffsetFromJointOne = new Vector3d(0.0, 0.0, -0.1);
      Vector3d sensorOffsetFromJointTwo = new Vector3d(0.0, 0.0, -0.1);

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis.Y, robot);
      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", sensorOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);

      PinJoint pinJointTwo = createPinJointWithHangingMass("pinJointTwo", massTwo, Axis.X, robot);
      JointWrenchSensor jointWrenchSensorTwo = new JointWrenchSensor("jointWrenchSensorTwo", sensorOffsetFromJointTwo, robot);
      pinJointTwo.addJointWrenchSensor(jointWrenchSensorTwo);
      assertTrue(jointWrenchSensorTwo == pinJointTwo.getJointWrenchSensor());
      pinJointOne.addJoint(pinJointTwo);

      robot.doDynamicsButDoNotIntegrate();

      Vector3d expectedJointForce = new Vector3d(0.0, 0.0, (massOne + massTwo) * robot.getGravityZ());
      Vector3d expectedJointTorque = new Vector3d();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3d(0.0, 0.0, (massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3d();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);

      assertJointWrenchSensorConsistency(robot, jointWrenchSensorOne);
      assertJointWrenchSensorConsistency(robot, jointWrenchSensorTwo);
      
      pinJointOne.setQ(Math.PI);
      robot.doDynamicsButDoNotIntegrate();

      expectedJointForce = new Vector3d(0.0, 0.0, -(massOne + massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3d();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3d(0.0, 0.0, -(massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3d();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testJointTorquesMatchWhenSensorAtJoint() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3d sensorOffsetFromJointOne = new Vector3d(0.0, 0.017, 0.0);
      Vector3d sensorOffsetFromJointTwo = new Vector3d(0.015, 0.0, 0.0);

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis.Y, robot);
      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", sensorOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);

      PinJoint pinJointTwo = createPinJointWithHangingMass("pinJointTwo", massTwo, Axis.X, robot);
      JointWrenchSensor jointWrenchSensorTwo = new JointWrenchSensor("jointWrenchSensorTwo", sensorOffsetFromJointTwo, robot);
      pinJointTwo.addJointWrenchSensor(jointWrenchSensorTwo);
      assertTrue(jointWrenchSensorTwo == pinJointTwo.getJointWrenchSensor());
      pinJointOne.addJoint(pinJointTwo);
      
      Tuple3d jointTorque = new Vector3d();
      
      Random random = new Random(1797L);
      pinJointOne.setQ(RandomTools.generateRandomDoubleInRange(random, -Math.PI, Math.PI));
      pinJointTwo.setQ(RandomTools.generateRandomDoubleInRange(random, -Math.PI, Math.PI));
      
      pinJointOne.setQd(RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0));
      pinJointTwo.setQd(RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0));
      
      for (int i=0; i<100; i++)
      {
         pinJointOne.setTau(RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0));
         pinJointTwo.setTau(RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0));
         
         robot.doDynamicsAndIntegrate(0.0001);
         
         jointWrenchSensorOne.getJointTorque(jointTorque);
         assertEquals(pinJointOne.getTauYoVariable().getDoubleValue(), -jointTorque.getY(), 1e-7);
         
         jointWrenchSensorTwo.getJointTorque(jointTorque);
         assertEquals(pinJointTwo.getTauYoVariable().getDoubleValue(), -jointTorque.getX(), 1e-7);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOffsetAtCenterOfMassWithCantileveredBeam() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis.Y, robot);
      
      Vector3d comOffsetFromJointOne = new Vector3d();
      pinJointOne.getLink().getComOffset(comOffsetFromJointOne);
      
      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", comOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);
      
      pinJointOne.setQ(Math.PI/2.0);
      pinJointOne.setTau(massOne * robot.getGravityZ() * comOffsetFromJointOne.getZ());
      
      robot.doDynamicsAndIntegrate(0.0001);
      
      double jointAcceleration = pinJointOne.getQDDYoVariable().getDoubleValue();
      assertEquals(0.0, jointAcceleration, 1e-7);
      
      Vector3d expectedJointForce = new Vector3d(-massOne * robot.getGravityZ(), 0.0, 0.0);
      Vector3d expectedJointTorque = new Vector3d();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);
   }
   
   
   private void assertJointWrenchSensorConsistency(Robot robot, JointWrenchSensor jointWrenchSensor)
   {
      Tuple3d jointForce = new Vector3d();
      Tuple3d jointTorque = new Vector3d();

      jointWrenchSensor.getJointForce(jointForce);
      jointWrenchSensor.getJointTorque(jointTorque);

      String name = jointWrenchSensor.getName();

      DoubleYoVariable forceX = (DoubleYoVariable) robot.getVariable(name + "_fX");
      DoubleYoVariable forceY = (DoubleYoVariable) robot.getVariable(name + "_fY");
      DoubleYoVariable forceZ = (DoubleYoVariable) robot.getVariable(name + "_fZ");

      DoubleYoVariable torqueX = (DoubleYoVariable) robot.getVariable(name + "_tX");
      DoubleYoVariable torqueY = (DoubleYoVariable) robot.getVariable(name + "_tY");
      DoubleYoVariable torqueZ = (DoubleYoVariable) robot.getVariable(name + "_tZ");

      assertEquals(jointForce.getX(), forceX.getDoubleValue(), 1e-7);
      assertEquals(jointForce.getY(), forceY.getDoubleValue(), 1e-7);
      assertEquals(jointForce.getZ(), forceZ.getDoubleValue(), 1e-7);

      assertEquals(jointTorque.getX(), torqueX.getDoubleValue(), 1e-7);
      assertEquals(jointTorque.getY(), torqueY.getDoubleValue(), 1e-7);
      assertEquals(jointTorque.getZ(), torqueZ.getDoubleValue(), 1e-7);
   }


   private void assertJointWrenchEquals(JointWrenchSensor jointWrenchSensor, Vector3d expectedJointForce, Vector3d expectedJointTorque)
   {
      Tuple3d jointForce = new Vector3d();
      Tuple3d jointTorque = new Vector3d();

      jointWrenchSensor.getJointForce(jointForce);
      jointWrenchSensor.getJointTorque(jointTorque);

      JUnitTools.assertTuple3dEquals(expectedJointForce, jointForce, 1e-7);
      JUnitTools.assertTuple3dEquals(expectedJointTorque, jointTorque, 1e-7);
   }

   private PinJoint createPinJointWithHangingMass(String name, double mass, Axis axis, Robot robot)
   {
      PinJoint pinJoint = new PinJoint(name, new Vector3d(), robot, axis);

      Vector3d comOffset = new Vector3d(0.0, 0.0, -1.0);
      Link linkOne = new Link("link");
      linkOne.setMass(mass);
      linkOne.setComOffset(comOffset);
      linkOne.setMassAndRadiiOfGyration(mass, 0.1, 0.1, 0.1);
      pinJoint.setLink(linkOne);

      return pinJoint;
   }

}
