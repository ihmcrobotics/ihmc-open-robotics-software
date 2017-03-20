package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class JointWrenchSensorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testStaticallyHangingMasses() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3D sensorOffsetFromJointOne = new Vector3D(0.0, 0.0, -0.1);
      Vector3D sensorOffsetFromJointTwo = new Vector3D(0.0, 0.0, -0.1);

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

      Vector3D expectedJointForce = new Vector3D(0.0, 0.0, (massOne + massTwo) * robot.getGravityZ());
      Vector3D expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3D(0.0, 0.0, (massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);

      assertJointWrenchSensorConsistency(robot, jointWrenchSensorOne);
      assertJointWrenchSensorConsistency(robot, jointWrenchSensorTwo);
      
      pinJointOne.setQ(Math.PI);
      robot.doDynamicsButDoNotIntegrate();

      expectedJointForce = new Vector3D(0.0, 0.0, -(massOne + massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3D(0.0, 0.0, -(massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testJointTorquesMatchWhenSensorAtJoint() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3D sensorOffsetFromJointOne = new Vector3D(0.0, 0.017, 0.0);
      Vector3D sensorOffsetFromJointTwo = new Vector3D(0.015, 0.0, 0.0);

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
      
      Tuple3DBasics jointTorque = new Vector3D();
      
      Random random = new Random(1797L);
      pinJointOne.setQ(RandomNumbers.nextDouble(random, -Math.PI, Math.PI));
      pinJointTwo.setQ(RandomNumbers.nextDouble(random, -Math.PI, Math.PI));
      
      pinJointOne.setQd(RandomNumbers.nextDouble(random, -1.0, 1.0));
      pinJointTwo.setQd(RandomNumbers.nextDouble(random, -1.0, 1.0));
      
      for (int i=0; i<100; i++)
      {
         pinJointOne.setTau(RandomNumbers.nextDouble(random, -1.0, 1.0));
         pinJointTwo.setTau(RandomNumbers.nextDouble(random, -1.0, 1.0));
         
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
      
      Vector3D comOffsetFromJointOne = new Vector3D();
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
      
      Vector3D expectedJointForce = new Vector3D(-massOne * robot.getGravityZ(), 0.0, 0.0);
      Vector3D expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);
   }
   
   
   private void assertJointWrenchSensorConsistency(Robot robot, JointWrenchSensor jointWrenchSensor)
   {
      Tuple3DBasics jointForce = new Vector3D();
      Tuple3DBasics jointTorque = new Vector3D();

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


   private void assertJointWrenchEquals(JointWrenchSensor jointWrenchSensor, Vector3D expectedJointForce, Vector3D expectedJointTorque)
   {
      Tuple3DBasics jointForce = new Vector3D();
      Tuple3DBasics jointTorque = new Vector3D();

      jointWrenchSensor.getJointForce(jointForce);
      jointWrenchSensor.getJointTorque(jointTorque);

      EuclidCoreTestTools.assertTuple3DEquals(expectedJointForce, jointForce, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(expectedJointTorque, jointTorque, 1e-7);
   }

   private PinJoint createPinJointWithHangingMass(String name, double mass, Axis axis, Robot robot)
   {
      PinJoint pinJoint = new PinJoint(name, new Vector3D(), robot, axis);

      Vector3D comOffset = new Vector3D(0.0, 0.0, -1.0);
      Link linkOne = new Link("link");
      linkOne.setMass(mass);
      linkOne.setComOffset(comOffset);
      linkOne.setMassAndRadiiOfGyration(mass, 0.1, 0.1, 0.1);
      pinJoint.setLink(linkOne);

      return pinJoint;
   }

}
