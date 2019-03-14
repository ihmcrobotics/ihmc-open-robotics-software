package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

public class DefaultFootstepPlannerParameters implements FootstepPlannerParameters
{
   /** {@inheritDoc} */
   @Override
   public double getMaximumStepReach()
   {
      return 0.45;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepWidth()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepCycleDistance()
   {
      return 0.6;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepLength()
   {
      return -0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepWidth()
   {
      return -0.15;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepYaw()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepYaw()
   {
      return -0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return 0.35;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepCycleChangeZ()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return 0.35;
   }

   public double getForwardWeight()
   {
      return 1.0;
   }

   public double getLateralWeight()
   {
      return 1.0;
   }

   public double getYawWeight()
   {
      return 0.1;
   }

   public double getCostPerStep()
   {
      return 0.1;
   }

   public double getStepUpWeight()
   {
      return 0.0;
   }

   public double getStepDownWeight()
   {
      return 0.0;
   }

   public double getHeuristicsWeight()
   {
      return 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistance()
   {
      return 0.02;
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
