package us.ihmc.robotics.kinematics;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RandomRestartInverseKinematicsCalculator implements InverseKinematicsCalculator
{
   private final Random random = new Random(1984L);
   private final NumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   private final OneDoFJoint[] joints;;
   private final int maxRestarts;
   private final double restartTolerance; 
   
   private double bestErrorScalar;
   private final DenseMatrix64F best = new DenseMatrix64F(1, 1);

   public RandomRestartInverseKinematicsCalculator(int maxRestarts, double restartTolerance, GeometricJacobian jacobian, NumericalInverseKinematicsCalculator inverseKinematicsCalculator)
   {
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      this.maxRestarts = maxRestarts;
      this.restartTolerance = restartTolerance;
   }
   
   @Override
   public boolean solve(RigidBodyTransform desiredTransform)
   {
      bestErrorScalar = Double.POSITIVE_INFINITY;
      
      boolean foundSolution = false;
      int restart = 0;
      
      while(restart < maxRestarts && !foundSolution)
      {
         foundSolution = inverseKinematicsCalculator.solve(desiredTransform);
         
         double errorScalar = inverseKinematicsCalculator.getErrorScalar();
         foundSolution = foundSolution || (errorScalar < restartTolerance);
         
         if (errorScalar < bestErrorScalar)
         {
            bestErrorScalar = errorScalar;
            inverseKinematicsCalculator.getBest(best);
         }
         
         if (!foundSolution)
         {
            ScrewTestTools.setRandomPositions(joints, random, -Math.PI/2.0, Math.PI/2.0);
         }
         
         restart++;
      }
      
      
      inverseKinematicsCalculator.setJointAngles(best);
      return foundSolution;
   }

   @Override
   public double getErrorScalar()
   {
      return inverseKinematicsCalculator.getErrorScalar();
   }

   @Override
   public int getNumberOfIterations()
   {
      return inverseKinematicsCalculator.getNumberOfIterations();
   }

   @Override
   public void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener)
   {
      inverseKinematicsCalculator.attachInverseKinematicsStepListener(stepListener);
   }

   @Override
   public void setLimitJointAngles(boolean limitJointAngles)
   {
      inverseKinematicsCalculator.setLimitJointAngles(limitJointAngles);
   }

}
