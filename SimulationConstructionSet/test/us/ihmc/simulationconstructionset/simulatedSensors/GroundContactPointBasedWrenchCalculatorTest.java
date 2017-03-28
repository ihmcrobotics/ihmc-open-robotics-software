package us.ihmc.simulationconstructionset.simulatedSensors;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
      
      GroundContactPoint point0 = new GroundContactPoint("point0", new Vector3D(), robot.getRobotsYoVariableRegistry());
      GroundContactPoint point1 = new GroundContactPoint("point1", new Vector3D(), robot.getRobotsYoVariableRegistry());

      List<GroundContactPoint> contactPoints = new ArrayList<GroundContactPoint>();
      contactPoints.add(point0);
      contactPoints.add(point1);
            
      joint = new PinJoint("test", new Vector3D(1.0, 0.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint);
      robot.update();
      
      
      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint, new RigidBodyTransform(), new YoVariableRegistry("dummy1"));
      
      
      point0.setForce(new Vector3D(0.0, 0.0, 1.0));
      point1.setForce(new Vector3D(0.0, 0.0, 0.0));
      
      point0.getYoPosition().set(new Point3D(1.0, 1.0, 0.0));
      point1.getYoPosition().set(new Point3D(-1.0, 0.0, 0.0));
      
      calculator.calculate();
      
      DenseMatrix64F tauXFXAndFZ = calculator.getWrench();
      assertEquals(1.0, tauXFXAndFZ.get(0, 0), epsilon);
      assertEquals(0.0, tauXFXAndFZ.get(3, 0), epsilon);
      assertEquals(1.0, tauXFXAndFZ.get(5, 0), epsilon);
      
      PinJoint joint2 = new PinJoint("test2", new Vector3D(-1.0, -1.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint2);
      robot.update();
      

      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint2, new RigidBodyTransform(), new YoVariableRegistry("dummy2"));
      point0.setForce(new Vector3D(-1.0, 1.0, 0.0));
      point1.setForce(new Vector3D(-1.0, 1.0, 0.0));
      
      point0.getYoPosition().set(new Point3D(0.0, 0.0, 1.0));
      point1.getYoPosition().set(new Point3D(-2.0, -2.0, 1.0));
      
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
      PinJoint joint3 = new PinJoint("test3", new Vector3D(0.0, 0.0, 0.0), robot, Axis.X);
      robot.addRootJoint(joint3);
      robot.update();
      
      RigidBodyTransform transformToJoint = new RigidBodyTransform();
      transformToJoint.setTranslation(new Vector3D(-1.0, -1.0, 0.0));

      calculator = new GroundContactPointBasedWrenchCalculator(joint.getName(), contactPoints, joint3, transformToJoint, new YoVariableRegistry("dummy3"));

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
