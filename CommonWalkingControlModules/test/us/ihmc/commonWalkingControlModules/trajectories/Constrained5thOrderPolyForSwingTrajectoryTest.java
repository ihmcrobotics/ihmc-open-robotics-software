package us.ihmc.commonWalkingControlModules.trajectories;

import org.junit.Test;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import static org.junit.Assert.assertTrue;

public class Constrained5thOrderPolyForSwingTrajectoryTest
{
   
   @Test
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
      
      System.out.print(X0);
      System.out.print("\n");
      
      trajectory.computeTrajectory(T0);
      assertTrue(Math.abs(trajectory.getPosition()-X0) < 0.000001);
      
      trajectory.computeTrajectory(Tf);
      assertTrue(Math.abs(trajectory.getPosition()-Xf) < 0.000001);
      System.out.print(trajectory.getVelocity());
      assertTrue(Math.abs(trajectory.getVelocity()-Vf) < 0.000001);
      
      trajectory.computeTrajectory((Tf+T0)/2);
      assertTrue(Math.abs(trajectory.getPosition()-Hmax) < 0.000001);
   }

}
