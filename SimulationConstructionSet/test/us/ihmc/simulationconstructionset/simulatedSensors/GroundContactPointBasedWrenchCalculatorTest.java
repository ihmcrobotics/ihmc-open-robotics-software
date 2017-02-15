package us.ihmc.simulationconstructionset.simulatedSensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class GroundContactPointBasedWrenchCalculatorTest
{
   WrenchCalculatorInterface calculator;
   OneDegreeOfFreedomJoint joint;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testWrenchCalculation()
   {
      double epsilon = 1e-7;
      Robot robot = new Robot("testRobot");
      
      GroundContactPoint point0 = new GroundContactPoint("point0", new Vector3d(), robot.getRobotsYoVariableRegistry());
      GroundContactPoint point1 = new GroundContactPoint("point1", new Vector3d(), robot.getRobotsYoVariableRegistry());

      List<GroundContactPoint> contactPoints = new ArrayList<GroundContactPoint>();
      contactPoints.add(point0);
      contactPoints.add(point1);
            
      joint = new PinJoint("test", new Vector3d(1.0, 0.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint);
      robot.update();
      
      
      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint, new RigidBodyTransform());
      
      
      point0.setForce(new Vector3d(0.0, 0.0, 1.0));
      point1.setForce(new Vector3d(0.0, 0.0, 0.0));
      
      point0.getYoPosition().set(new Point3d(1.0, 1.0, 0.0));
      point1.getYoPosition().set(new Point3d(-1.0, 0.0, 0.0));
      
      calculator.calculate();
      
      DenseMatrix64F tauXFXAndFZ = calculator.getWrench();
      assertEquals(1.0, tauXFXAndFZ.get(0, 0), epsilon);
      assertEquals(0.0, tauXFXAndFZ.get(3, 0), epsilon);
      assertEquals(1.0, tauXFXAndFZ.get(5, 0), epsilon);
      
      PinJoint joint2 = new PinJoint("test2", new Vector3d(-1.0, -1.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint2);
      robot.update();
      

      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint2, new RigidBodyTransform());
      point0.setForce(new Vector3d(-1.0, 1.0, 0.0));
      point1.setForce(new Vector3d(-1.0, 1.0, 0.0));
      
      point0.getYoPosition().set(new Point3d(0.0, 0.0, 1.0));
      point1.getYoPosition().set(new Point3d(-2.0, -2.0, 1.0));
      
      calculator.calculate();
      
      DenseMatrix64F wholeWrench = calculator.getWrench();
      assertTrue(wholeWrench.getNumRows() == 6);
      assertEquals(wholeWrench.get(0,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(1,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(2,0), 0.0, epsilon);
      assertEquals(wholeWrench.get(3,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(4,0), 2.0, epsilon);
      assertEquals(wholeWrench.get(5,0), 0.0, epsilon);
      
      
      
      //Off-joint-axis force-Sensor
      PinJoint joint3 = new PinJoint("test3", new Vector3d(0.0, 0.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint3);
      robot.update();
      
      RigidBodyTransform transformToJoint = new RigidBodyTransform();
      transformToJoint.setTranslation(new Vector3d(-1.0, -1.0, 0.0));

      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint3, transformToJoint);

      calculator.calculate();
      wholeWrench = calculator.getWrench();
      assertTrue(wholeWrench.getNumRows() == 6);
      assertEquals(wholeWrench.get(0,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(1,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(2,0), 0.0, epsilon);
      assertEquals(wholeWrench.get(3,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(4,0), 2.0, epsilon);
      assertEquals(wholeWrench.get(5,0), 0.0, epsilon);

      //different joint angle, which also changed sensor frame
      joint3.setQ(Math.PI);
      robot.update();

      calculator.calculate();
      wholeWrench = calculator.getWrench();
      assertTrue(wholeWrench.getNumRows() == 6);
      assertEquals(wholeWrench.get(0,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(1,0), + 2.0, epsilon);
      assertEquals(wholeWrench.get(2,0), + 4.0, epsilon);
      assertEquals(wholeWrench.get(3,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(4,0), - 2.0, epsilon);
      assertEquals(wholeWrench.get(5,0), 0.0, epsilon);
   }
}
