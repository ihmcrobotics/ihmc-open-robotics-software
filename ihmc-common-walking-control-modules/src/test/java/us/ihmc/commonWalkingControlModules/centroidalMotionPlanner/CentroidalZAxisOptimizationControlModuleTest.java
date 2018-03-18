package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import java.util.List;

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
                                                                                                            new OptimizationControlModuleHelper(parameters),
                                                                                                            parameters);
      List<Trajectory> forceTrajectory = controlModule.getForceProfile();
      List<Trajectory> positionTrajectory= controlModule.getHeightTrajectory();
      List<Trajectory> linearVelocityTrajectory = controlModule.getLinearVelocityProfile();
      for (int i = 0; i < forceTrajectory.size(); i++)
         assertTrue(forceTrajectory.get(i).getNumberOfCoefficients() != 0);
      for (int i = 0; i < positionTrajectory.size(); i++)
         assertTrue(positionTrajectory.get(i).getNumberOfCoefficients() != 0);
      for (int i = 0; i < linearVelocityTrajectory.size(); i++)
         assertTrue(linearVelocityTrajectory.get(i).getNumberOfCoefficients() != 0);
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