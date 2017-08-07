package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;



public class KinematicPointTest 
{
	Vector3D offset;
	Robot robot;
	KinematicPoint kinematicPoint;

	@Before
	public void setUp()
	{
		offset = new Vector3D(1.0, 2.0, 3.0);
		robot = new Robot("testRobot");
		kinematicPoint = new KinematicPoint("testPoint", offset, robot.getRobotsYoVariableRegistry());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetAndSetParentJoint() 
	{
		PinJoint joint = new PinJoint("joint", new Vector3D(0.0, 0.0, 0.0), robot, Axis.X);
		robot.addRootJoint(joint);
		kinematicPoint.setParentJoint(joint);
		assertTrue(joint == kinematicPoint.getParentJoint());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testToString()
	{
		assertEquals("name: testPoint x: 0.0, y: 0.0, z: 0.0", kinematicPoint.toString());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testSetOffsetJointWithBothVectorAndXYAndZValuesAsParameters()
	{
	   Robot robot = new Robot("testRobot");
      KinematicPoint kinematicPoint = new KinematicPoint("testPoint", robot.getRobotsYoVariableRegistry());
      
		assertTrue(0.0 == kinematicPoint.getOffsetCopy().getX());
		assertTrue(0.0 == kinematicPoint.getOffsetCopy().getY());
		assertTrue(0.0 == kinematicPoint.getOffsetCopy().getZ());

		kinematicPoint.setOffsetJoint(3.0, 4.0, 7.0);
		assertTrue(3.0 == kinematicPoint.getOffsetCopy().getX());
		assertTrue(4.0 == kinematicPoint.getOffsetCopy().getY());
		assertTrue(7.0 == kinematicPoint.getOffsetCopy().getZ());
		
		Vector3D vectorTest = new Vector3D(9.0, 1.0, 5.0);
		kinematicPoint.setOffsetJoint(vectorTest);
		assertTrue(9.0 == kinematicPoint.getOffsetCopy().getX());
		assertTrue(1.0 == kinematicPoint.getOffsetCopy().getY());
		assertTrue(5.0 == kinematicPoint.getOffsetCopy().getZ());
	}
	
//	@Test(timeout=300000)
//	public void testSetOffsetWorld()
//	{
//		kinematicPoint.setOffsetWorld(4.0, 1.5, 3.5);
//		assertTrue(4.0 == kinematicPoint.getOffset().getX());
//		assertTrue(1.5 == kinematicPoint.getOffset().getY());
//		assertTrue(3.5 == kinematicPoint.getOffset().getZ());
//	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetName()
	{
		assertTrue(kinematicPoint.getName() == "testPoint");
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetPosition()
	{
		Point3D positionToPack = new Point3D();
		kinematicPoint.getPosition(positionToPack);
		assertTrue(0 == positionToPack.getX());
		assertTrue(0 == positionToPack.getY());
		assertTrue(0 == positionToPack.getZ());
		
		kinematicPoint.getYoPosition().set(new Point3D(5.0, 5.1, 5.2));
		kinematicPoint.getPosition(positionToPack);
		assertTrue(5.0 == positionToPack.getX());
		assertTrue(5.1 == positionToPack.getY());
		assertTrue(5.2 == positionToPack.getZ());

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetPositionPoint()
	{
		Point3D positionReceivedFromGetMethod = kinematicPoint.getPositionPoint();
		assertTrue(0 == positionReceivedFromGetMethod.getX());
		assertTrue(0 == positionReceivedFromGetMethod.getY());
		assertTrue(0 == positionReceivedFromGetMethod.getZ());
		
		kinematicPoint.getYoPosition().set(new Point3D(5.0, 5.1, 5.2));
		positionReceivedFromGetMethod = kinematicPoint.getPositionPoint();
		assertTrue(5.0 == positionReceivedFromGetMethod.getX());
		assertTrue(5.1 == positionReceivedFromGetMethod.getY());
		assertTrue(5.2 == positionReceivedFromGetMethod.getZ());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetVelocityVector()
	{
		Vector3D vectorReceivedFromGetMethod = kinematicPoint.getVelocityVector();
		assertTrue(0 == vectorReceivedFromGetMethod.getX());
		assertTrue(0 == vectorReceivedFromGetMethod.getY());
		assertTrue(0 == vectorReceivedFromGetMethod.getZ());
		
		kinematicPoint.getYoVelocity().set(new Vector3D(5.0, 5.1, 5.2));
		vectorReceivedFromGetMethod = kinematicPoint.getVelocityVector();
		assertTrue(5.0 == vectorReceivedFromGetMethod.getX());
		assertTrue(5.1 == vectorReceivedFromGetMethod.getY());
		assertTrue(5.2 == vectorReceivedFromGetMethod.getZ());
		
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetVelocity()
	{
		Vector3D velocityToPack = kinematicPoint.getVelocityVector();
		kinematicPoint.getVelocity(velocityToPack);
		assertTrue(0 == velocityToPack.getX());
		assertTrue(0 == velocityToPack.getY());
		assertTrue(0 == velocityToPack.getZ());
		
		kinematicPoint.getYoVelocity().set(new Vector3D(5.0, 5.1, 5.2));
		kinematicPoint.getVelocity(velocityToPack);
		assertTrue(5.0 == velocityToPack.getX());
		assertTrue(5.1 == velocityToPack.getY());
		assertTrue(5.2 == velocityToPack.getZ());
		
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetYoPosition()
	{
		YoFramePoint yoPosition = kinematicPoint.getYoPosition();
		String frameName = yoPosition.getReferenceFrame().getName();

		assertEquals("( 0.000,  0.000,  0.000 )-" + frameName, yoPosition.toString());
		yoPosition.set(new Point3D(5.0, 5.1, 5.2));
		assertEquals("( 5.000,  5.100,  5.200 )-" + frameName, yoPosition.toString());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testGetYoVelocity()
	{
		YoFrameVector yoVelocity = kinematicPoint.getYoVelocity();
		String frameName = yoVelocity.getReferenceFrame().getName();
		
      assertEquals("( 0.000,  0.000,  0.000 )-" + frameName, yoVelocity.toString());
		yoVelocity.set(new Vector3D(5.0, 5.1, 5.2));
		assertEquals("( 5.000,  5.100,  5.200 )-" + frameName, yoVelocity.toString());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testChangeableOffset()
	{
	   Robot robot = new Robot("testRobot");
	   
	   KinematicPoint kinematicPoint = new KinematicPoint("kp_test", robot.getRobotsYoVariableRegistry());
	   
	   Vector3D offset = new Vector3D(0.1, 0.2, 0.3);
	   kinematicPoint.setOffsetJoint(offset);
	   
	   Vector3D offsetCopy = kinematicPoint.getOffsetCopy();
	   
	   EuclidCoreTestTools.assertTuple3DEquals(offset, offsetCopy, 1e-14);
	}

}
