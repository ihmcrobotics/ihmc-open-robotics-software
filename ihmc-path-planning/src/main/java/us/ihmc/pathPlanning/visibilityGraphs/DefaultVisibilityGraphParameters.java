package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;

public class DefaultVisibilityGraphParameters implements VisibilityGraphsParameters
{

   /** {@inheritDoc} */
   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return 0.55;
   }

   /** {@inheritDoc} */
   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return Math.cos(Math.toRadians(45.0));
   }

   /** {@inheritDoc} */
   @Override
   public double getPreferredObstacleExtrusionDistance()
   {
      return 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getObstacleExtrusionDistance()
   {
      return 0.4;
   }

   /** {@inheritDoc} */
   @Override
   public double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return 0.05;
   }

   /** {@inheritDoc} */
   @Override
   public double getTooHighToStepDistance()
   {
      return 0.28;
   }

   /** {@inheritDoc} */
   @Override
   public double getClusterResolution()
   {
      return 0.2;
   }

   /** {@inheritDoc} */
   @Override
   public double getPlanarRegionMinArea()
   {
      return 0.05;
   }

   /** {@inheritDoc} */
   @Override
   public int getPlanarRegionMinSize()
   {
      return 2;
   }

}
