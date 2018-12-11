package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class ValkyrieFootstepPlannerParameters implements FootstepPlannerParameters
{
   @Override
   public boolean checkForBodyBoxCollisions()
   {
      return true;
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return 0.2;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return 0.2;
   }

   @Override
   public double getMaximumStepReach()
   {
      return 0.35;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return 0.6;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return -0.15;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return 0.2;
   }

   @Override
   public double getMaximumStepZ()
   {
      return 0.15;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return 0.4;
   }
   
   @Override
   public boolean getReturnBestEffortPlan()
   {
      return false;
   }

   @Override
   public double getBodyBoxBaseX()
   {
      return 0.0;
   }

   @Override
   public double getBodyBoxWidth()
   {
      return 0.85;
   }

   @Override
   public double getBodyBoxDepth()
   {
      return 0.4;
   }

   @Override
   public int getMinimumStepsForBestEffortPlan()
   {
      return 3;
   }

   @Override
   public double getMinXClearanceFromStance()
   {
      return 0.2;
   }

   @Override
   public double getMinYClearanceFromStance()
   {
      return 0.2;
   }

   @Override
   public FootstepPlannerCostParameters getCostParameters()
   {
      return new ValkyrieFootstepPlannerCostParameters();
   }
   
   @Override
   public double getCliffHeightToAvoid()
   {
      return 0.07;
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return 0.04;
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return 0.01;
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return true;
   }

   @Override
   public double getStepTranslationBoundingBoxScaleFactor()
   {
      return 0.65;
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return 0.04;
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return false;
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return 0.3;
   }
}
