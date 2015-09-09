package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class Constrained5thOrderPolyForSwingTrajectoryTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void TestConstrainedCubicForSwingTrajectory()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      String name = "TestName";
      
      Constrained5thOrderPolyForSwingFootTrajectory trajectory = new Constrained5thOrderPolyForSwingFootTrajectory(name, registry);
      
      double X0 = 0.2;
      double Hmax = 0.9;
      double Xf = 1;
      double T0 = 0.2;
      double Tf = 1.2;
      double Vf = -0.3;
      
      trajectory.setParams(X0,Hmax, Xf, Vf, T0, Tf);
      
      trajectory.computeTrajectory(T0);
      assertTrue(Math.abs(trajectory.getPosition()-X0) < 0.000001);
      
      trajectory.computeTrajectory(Tf);
      assertTrue(Math.abs(trajectory.getPosition()-Xf) < 0.000001);
      assertTrue(Math.abs(trajectory.getVelocity()-Vf) < 0.000001);
      
      trajectory.computeTrajectory((Tf+T0)/2);
      assertTrue(Math.abs(trajectory.getPosition()-Hmax) < 0.000001);
   }

}
