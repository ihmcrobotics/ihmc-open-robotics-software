package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

public class DefaultFootstepPlannerParameters implements FootstepPlannerParameters
{
   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepReach()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLength()
   {
      return 0.45;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepOutward()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLength()
   {
      return -getMaximumFrontStepLength();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepInward()
   {
      return -0.15;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepYawOutward()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepYawInward()
   {
      return -0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return 0.35;
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return 0.35;
   }

   @Override
   public double getDistanceWeight()
   {
      return 1.0;
   }

   @Override
   public double getXGaitWeight()
   {
      return 7.5;
   }

   @Override
   public double getDesiredVelocityWeight()
   {
      return 1.0;
   }

   @Override
   public double getYawWeight()
   {
      return 2.5;
   }

   @Override
   public double getCostPerStep()
   {
      return 0.25;
   }

   @Override
   public double getStepUpWeight()
   {
      return 0.0;
   }

   @Override
   public double getStepDownWeight()
   {
      return 0.0;
   }

   @Override
   public double getHeuristicsInflationWeight()
   {
      return 1.75;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForExpansion()
   {
      return 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForPostProcessing()
   {
      return 0.04;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getProjectInsideUsingConvexHullDuringExpansion()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getProjectInsideUsingConvexHullDuringPostProcessing()
   {
      return true;
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return 0.03;
   }

   @Override
   public double getMinXClearanceFromFoot()
   {
      return 0.05;
   }

   @Override
   public double getMinYClearanceFromFoot()
   {
      return 0.05;
   }
}
