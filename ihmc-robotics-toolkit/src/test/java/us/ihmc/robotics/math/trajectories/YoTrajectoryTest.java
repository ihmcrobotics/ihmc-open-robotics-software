package us.ihmc.robotics.math.trajectories;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoTrajectoryTest
{
   YoVariableRegistry registry = new YoVariableRegistry("TrajectoryTestRegistry");

   @Before
   public void setup()
   {
      registry.clear();
   }

   @Test(timeout = 30000)
   public void testGetDerivative()
   {
      YoTrajectory traj = new YoTrajectory("Trajectory", 10, registry);
      YoTrajectory dervTraj = new YoTrajectory("DerivativeTrajectory", 9, registry);
      traj.setCubic(1, 10, 0, 8);
      traj.getDerivative(dervTraj, 1);
     
      assert (dervTraj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(1.0 * traj.getCoefficient(1), dervTraj.getCoefficient(0), 1));
      assert (MathTools.epsilonCompare(2.0 * traj.getCoefficient(2), dervTraj.getCoefficient(1), 1));
      assert (MathTools.epsilonCompare(3.0 * traj.getCoefficient(3), dervTraj.getCoefficient(2), 1));      
   }
}
