package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolverWithInactiveVariables;
import us.ihmc.euclid.tuple2D.Vector2D;

public class KinematicsToolboxOptimizationSettings implements ControllerCoreOptimizationSettings
{
   /** @inheritDoc */
   @Override
   public double getJointVelocityWeight()
   {
      return 0.00001;
   }

   /** @inheritDoc */
   @Override
   public double getJointAccelerationWeight()
   {
      return 0.0;
   }

   @Override
   public boolean areJointVelocityLimitsConsidered()
   {
      return false;
   }

   /** @inheritDoc */
   @Override
   public double getJointJerkWeight()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getRhoWeight()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getRhoMin()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateDefaultWeight()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateHighWeight()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPWeight()
   {
      return null;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return null;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return null;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactableBodies()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public int getRhoSize()
   {
      return 0;
   }

   public boolean getDeactivateRhoWhenNotInContact()
   {
      return true;
   }

   @Override
   public ActiveSetQPSolverWithInactiveVariablesInterface getActiveSetQPSolver()
   {
      return new JavaQuadProgSolverWithInactiveVariables();
   }
}
