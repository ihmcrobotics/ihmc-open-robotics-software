package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class AtlasFootstepPlannerParameters implements FootstepPlannerParameters
{

   @Override
   public double getIdealFootstepWidth()
   {
      return 0.22;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return 0.3;
   }

   @Override
   public double getMaximumStepReach()
   {
      return 0.4;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return 0.4;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaximumStepZ()
   {
      return 0.25;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return 0.4;
   }

   @Override
   public double getCliffHeightToAvoid()
   {
      return 0.05;
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return 0.1;
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return 0.03;
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return true;
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return false;
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return 0.05;
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return Math.toRadians(7.5);
   }
}
