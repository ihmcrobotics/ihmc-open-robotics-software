package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

public class DefaultFootstepPlannerParameters implements FootstepPlannerParameters
{
   /** {@inheritDoc} */
   @Override
   public double getMaximumStepReach()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepWidth()
   {
      return 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepCycleDistance()
   {
      return 0.7;
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
      return -0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepCycleChangeZ()
   {
      return 0.15;
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return 0.1;
   }
}
