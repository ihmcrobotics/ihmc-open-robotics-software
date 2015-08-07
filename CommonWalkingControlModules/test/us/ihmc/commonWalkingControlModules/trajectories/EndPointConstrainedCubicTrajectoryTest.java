package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class EndPointConstrainedCubicTrajectoryTest
{

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void TestEndPointConstrainedCubicTrajectory()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      String name = "TestName";
      
      EndPointConstrainedCubicTrajectory trajectory = new EndPointConstrainedCubicTrajectory(name, registry);
      
      double X0 = 0.2;
      double Xf = 1;
      double t0 = 0.2;
      double tf = 1.2;
      double Xdotf = -0.3;
      double Xdot0 = 0.1;
      
      trajectory.setParams(t0, tf, X0, Xdot0, Xf, Xdotf);
      
      trajectory.computeTrajectory(t0);
      assertTrue(Math.abs(trajectory.getPosition()-X0) < 0.000001);
      assertTrue(Math.abs(trajectory.getVelocity()-Xdot0) < 0.000001);
      
      trajectory.computeTrajectory(tf);
      assertTrue(Math.abs(trajectory.getPosition()-Xf) < 0.000001);
      assertTrue(Math.abs(trajectory.getVelocity()-Xdotf) < 0.000001);
   }

}
