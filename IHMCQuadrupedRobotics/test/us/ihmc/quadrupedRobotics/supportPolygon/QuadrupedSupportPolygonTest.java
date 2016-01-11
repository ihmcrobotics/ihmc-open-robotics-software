package us.ihmc.quadrupedRobotics.supportPolygon;

import static org.junit.Assert.assertTrue;
import static us.ihmc.tools.testing.TestPlanTarget.Fast;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = Fast)
public class QuadrupedSupportPolygonTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCreateSupportPolygonAndCallSomeStuffOnIt()
   {
      QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
      
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         quadrupedSupportPolygon.setFootstep(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3d tuple3dToPack = new Point3d();
         quadrupedSupportPolygon.getFootstep(robotQuadrant).get(tuple3dToPack);
         assertTrue("Point not equal", tuple3dToPack.equals(footPoints.get(robotQuadrant)));
      }
   }
}
