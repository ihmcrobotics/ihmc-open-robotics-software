package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoTrajectoryTest
{
   YoRegistry registry = new YoRegistry("TrajectoryTestRegistry");

   @BeforeEach
   public void setup()
   {
      registry.clear();
   }

   @Test
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
