package us.ihmc.commonWalkingControlModules.calculators;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.Robot;

public class WrenchComponentCalculatorTest
{
   WrenchComponentCalculator calculator;
   Transform3D transformToParent;
   ReferenceFrame originFrame;
   
   @Test
   public void testWrenchCalculation()
   {
      double epsilon = 1e-7;
      Robot robot = new Robot("testRobot");
      
      GroundContactPoint point0 = new GroundContactPoint("point0", new Vector3d(), robot);
      GroundContactPoint point1 = new GroundContactPoint("point1", new Vector3d(), robot);

      List<GroundContactPoint> contactPoints = new ArrayList<GroundContactPoint>();
      contactPoints.add(point0);
      contactPoints.add(point1);
      
      transformToParent = new Transform3D();
      transformToParent.set(new Vector3d(1.0, 0.0, 0.0));
      originFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("", ReferenceFrame.getWorldFrame(), transformToParent, false, false, true);
            
      calculator = new WrenchComponentCalculator(contactPoints, originFrame);
      
      point0.setForce(new Vector3d(0.0, 0.0, 1.0));
      point1.setForce(new Vector3d(0.0, 0.0, 0.0));
      
      point0.getYoPosition().set(new Point3d(1.0, 1.0, 0.0));
      point1.getYoPosition().set(new Point3d(-1.0, 0.0, 0.0));
      
      double[] tauXFXAndFZ = calculator.getWrenchComponents(new int[]{0, 3, 5});
      assertTrue(tauXFXAndFZ.length == 3);
      assertEquals(tauXFXAndFZ[0], 1.0, epsilon);
      assertEquals(tauXFXAndFZ[1], 0.0, epsilon);
      assertEquals(tauXFXAndFZ[2], 1.0, epsilon);
      
      transformToParent = new Transform3D();
      transformToParent.set(new Vector3d(-1.0, -1.0, 0.0));
      originFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("", ReferenceFrame.getWorldFrame(), transformToParent, false, false, true);

      calculator = new WrenchComponentCalculator(contactPoints, originFrame);
      
      point0.setForce(new Vector3d(-1.0, 1.0, 0.0));
      point1.setForce(new Vector3d(-1.0, 1.0, 0.0));
      
      point0.getYoPosition().set(new Point3d(0.0, 0.0, 1.0));
      point1.getYoPosition().set(new Point3d(-2.0, -2.0, 1.0));
      
      double[] wholeWrench = calculator.getWrenchComponents();
      assertTrue(wholeWrench.length == 6);
      assertEquals(wholeWrench[0], - 2.0, epsilon);
      assertEquals(wholeWrench[1], - 2.0, epsilon);
      assertEquals(wholeWrench[2], 0.0, epsilon);
      assertEquals(wholeWrench[3], - 2.0, epsilon);
      assertEquals(wholeWrench[4], 2.0, epsilon);
      assertEquals(wholeWrench[5], 0.0, epsilon);
   }
}
