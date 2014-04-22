package us.ihmc.commonWalkingControlModules.calculators;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.sensorProcessing.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.utilities.Axis;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class GroundContactPointBasedWrenchCalculatorTest
{
   WrenchCalculatorInterface calculator;
   OneDegreeOfFreedomJoint joint;
   
   @Test
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
      
      
      calculator = new GroundContactPointBasedWrenchCalculator(contactPoints, joint);
      
      
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
      

      calculator = new GroundContactPointBasedWrenchCalculator(contactPoints, joint2);
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
   }
}
