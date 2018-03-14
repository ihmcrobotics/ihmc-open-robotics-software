package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.robotics.math.trajectories.Trajectory;

public class CentroidalZAxisOptimizationControlModuleTest
{
   private final int maxNumberOfNodes = 10;
   private final double robotMass = 18.0;
   private final double deltaTMin = 0.001;
   private final double Izz = 0.056;

   @Test
   public void testGeneratesAPlan()
   {
      CentroidalZAxisOptimizationControlModule controlModule = new CentroidalZAxisOptimizationControlModule(robotMass);
      Trajectory[] trajectoryList = new Trajectory[3];
      trajectoryList[0] = controlModule.getForceProfile();
      trajectoryList[1] = controlModule.getHeightTrajectory();
      trajectoryList[2] = controlModule.getLinearVelocityProfile();
      for (int i = 0; i < trajectoryList.length; i++)
         assertTrue(trajectoryList[i].getNumberOfCoefficients() != 0);
   }

   @Test
   public void testPlanIsFeasible()
   {
      assertTrue(false);
   }

   @Test
   public void testPlanMeetsObjectives()
   {
      assertTrue(false);
   }
}