package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.TrajectoryMathClass;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoTrajectoryTest
{
   YoVariableRegistry registry = new YoVariableRegistry("TrajectoryTestRegistry");
   
   @Test
   public void testAddition()
   {      
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 1, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 1, registry);
      YoTrajectory traj3 = new YoTrajectory("Trajectory3", 1, registry);
      traj1.setConstant(1, 10, 2);
      traj2.setConstant(1, 10, 3);      
      TrajectoryMathClass.add(traj3, traj1, traj2);      
      assert(traj3.getNumberOfCoefficients() == 1);
      assert(traj3.getCoefficient(0) == 5);
      
      traj1 = new YoTrajectory("Trajectory4", 2, registry);
      traj1.setLinear(1, 10, 0, 2);
      traj1.add(traj2);
      assert(traj1.getNumberOfCoefficients() == 2);
      assert(traj1.getCoefficient(0) == 3 - 2.0/9.0);
      assert(traj1.getCoefficient(1) == 2.0/9.0);
   }
   
   @Test
   public void testMultiply()
   {      
      YoTrajectory traj1 = new YoTrajectory("Trajectory1", 2, registry);
      YoTrajectory traj2 = new YoTrajectory("Trajectory2", 1, registry);
      YoTrajectory traj3 = new YoTrajectory("Trajectory3", 2, registry);
      traj1.setLinear(1, 10, 2, 0);
      traj2.setConstant(1, 10, 3);
      TrajectoryMathClass.multiply(traj3, traj1, traj2);   
      assert(traj3.getNumberOfCoefficients() == 2);
      assert(traj3.getCoefficient(1) == -2.0/3.0);
      assert(traj3.getCoefficient(0) == 6 + 2.0/3.0);    
      
      traj1 = new YoTrajectory("Trajectory4", 10, registry);
      traj2 = new YoTrajectory("Trajectory5", 10, registry);
      traj1.setCubic(1, 10, 1, 10);
      traj2.setCubic(1, 10, 5, -2);
      PrintTools.info(traj1.toString());
      PrintTools.info(traj2.toString());
      traj1.multiply(traj2);
      PrintTools.info(traj1.toString());  
      assert(traj1.getNumberOfCoefficients() == 6);
      assert(traj1.getCoefficient(0) == 0);
      assert(traj1.getCoefficient(1) == 0);
      assert(traj1.getCoefficient(2) == 0);
      assert(traj1.getCoefficient(3) == 0);
      assert(traj1.getCoefficient(4) == 0);
      assert(traj1.getCoefficient(5) == 0);
   }
}
