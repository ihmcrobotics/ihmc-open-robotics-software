package us.ihmc.commonWalkingControlModules.trajectories;

import org.junit.Test;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import static org.junit.Assert.assertTrue;

public class ConstrainedCubicForSwingTrajectoryTest
{
   
   @Test
   public void TestConstrainedCubicForSwingTrajectory()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      String name = "TestName";
      
      ConstrainedCubicForSwingFootTrajectory trajectory = new ConstrainedCubicForSwingFootTrajectory(name, registry);
      
      double X0 = 0.2;
      double Hmax = 0.9;
      double Xf = 1;
      double T0 = 0.2;
      double Tf = 1.2;
      
      trajectory.setParams(X0,Hmax, Xf, T0, Tf);
      
      System.out.print(X0);
      System.out.print("\n");
      
      trajectory.computeTrajectory(T0);
      System.out.print(trajectory.getPosition());
      assertTrue(Math.abs(trajectory.getPosition()-X0) < 0.0000001);
      
      trajectory.computeTrajectory(Tf);
      assertTrue(Math.abs(trajectory.getPosition()-Xf) < 0.0000001);
      
      trajectory.computeTrajectory((Tf+T0)/2);
      assertTrue(Math.abs(trajectory.getPosition()-Hmax) < 0.0000001);
   }

}
