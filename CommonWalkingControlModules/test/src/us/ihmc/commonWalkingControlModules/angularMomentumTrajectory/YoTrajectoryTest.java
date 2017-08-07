package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.TrajectoryMathTools;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoTrajectoryTest
{
   YoVariableRegistry registry = new YoVariableRegistry("TrajectoryTestRegistry");

   @Before
   public void setup()
   {
      registry.clear();
   }

   @Test
   public void testAddition()
   {
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 1, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 1, registry);
      YoTrajectory traj3 = new YoTrajectory("Trajectory3", 1, registry);
      traj1.setConstant(1, 10, 2);
      traj2.setConstant(1, 10, 3);
      TrajectoryMathTools.add(traj3, traj1, traj2);
      assert (traj3.getNumberOfCoefficients() == 1);
      assert (traj3.getCoefficient(0) == 5);

      traj1 = new YoTrajectory("Trajectory4", 2, registry);
      traj1.setLinear(1, 10, 0, 2);
      traj1.add(traj2);
      assert (traj1.getNumberOfCoefficients() == 2);
      assert (traj1.getCoefficient(0) == 3 - 2.0 / 9.0);
      assert (traj1.getCoefficient(1) == 2.0 / 9.0);
   }

   @Test
   public void testGetDerivative()
   {
      YoTrajectory traj = new YoTrajectory("Trajectory", 10, registry);
      YoTrajectory dervTraj = new YoTrajectory("DerivativeTrajectory", 9, registry);
      traj.setCubic(1, 10, 0, 8);
      traj.getDerivative(dervTraj);
     
      assert (dervTraj.getNumberOfCoefficients() == 3);
      assert (MathTools.epsilonCompare(1.0 * traj.getCoefficient(1), dervTraj.getCoefficient(0), 1));
      assert (MathTools.epsilonCompare(2.0 * traj.getCoefficient(2), dervTraj.getCoefficient(1), 1));
      assert (MathTools.epsilonCompare(3.0 * traj.getCoefficient(3), dervTraj.getCoefficient(2), 1));      
   }
}
