package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.robotics.math.trajectories.Trajectory;

public class CentroidalZAxisOptimizationControlModuleTest
{
   private static final double robotMass = 18.0;
   private static final double deltaTMin = 0.001;
   private static final double Izz = 0.056;
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();

   static
   {
      parameters.setRobotMass(robotMass);
      parameters.setDeltaTMin(deltaTMin);
      parameters.setNominalIzz(Izz);
   }

   @Test
   public void testGeneratesAPlan()
   {
      CentroidalZAxisOptimizationControlModule controlModule = new CentroidalZAxisOptimizationControlModule(robotMass,
                                                                                                            new OptimizationControlModuleHelper(parameters));
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