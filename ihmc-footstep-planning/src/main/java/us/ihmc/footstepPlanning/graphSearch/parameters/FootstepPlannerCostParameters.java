package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.stepCost.QuadraticDistanceAndYawCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.LinearHeightCost;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface FootstepPlannerCostParameters
{
   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
    */
   default boolean useQuadraticDistanceCost()
   {
      return false;
   }

   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
    */
   default boolean useQuadraticHeightCost()
   {
      return false;
   }

   /**
    * Gets the weight for the heuristics in the A Star planner.
    */
   default DoubleProvider getAStarHeuristicsWeight()
   {
      return () -> 1.5;
   }

   /**
    * Gets the weight for the heuristics in the Visibility graph with A star planner.
    */
   default DoubleProvider getVisGraphWithAStarHeuristicsWeight()
   {
      return () -> 15.0;
   }

   /**
    * Gets the weight for the heuristics in the Depth First planner.
    */
   default DoubleProvider getDepthFirstHeuristicsWeight()
   {
      return () -> 1.0;
   }

   /**
    * Gets the weight for the heuristics in the Body path based planner.
    */
   default DoubleProvider getBodyPathBasedHeuristicsWeight()
   {
      return () -> 1.0;
   }

   /**
    * When using a cost based planning approach this value defined how the yaw of a footstep will be
    * weighted in comparison to its position.
    */
   default double getYawWeight()
   {
      return 0.1;
   }

   /**
    * <p>
    * This value defined how the forward (or backward) displacement of a footstep will be weighted in
    * comparison to its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getLateralWeight()}
    * </p>
    */
   default double getForwardWeight()
   {
      return 1.0;
   }

   /**
    * <p>
    * This value defined how the lateral displacement of a footstep will be weighted in comparison to
    * its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getForwardWeight()}
    * </p>
    */
   default double getLateralWeight()
   {
      return 1.0;
   }

   /**
    * When using a cost based planning approach this value defines the cost that is added for each step
    * taken. Setting this value to a high number will favor plans with less steps.
    */
   default double getCostPerStep()
   {
      return 0.15;
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * up will be weighted.
    */
   default double getStepUpWeight()
   {
      return 0.0;
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * down will be weighted.
    */
   default double getStepDownWeight()
   {
      return 0.0;
   }

   /**
    * When using a cost based planning approach this value defines how the roll will be weighted.
    */
   default double getRollWeight()
   {
      return 0.0;
   }

   /**
    * When using a cost based planning approach this value defines how the pitch will be weighted.
    */
   default double getPitchWeight()
   {
      return 0.0;
   }

   /**
    * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
    * @see FootstepPlannerCostParameters#getBoundingBoxCost
    */
   default double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return 0.0;
   }

   /**
    * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
    * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
    * {@code c * (1 - d / d_max)}, where d_max is this value.
    */
   default double getBoundingBoxCost()
   {
      return 0.0;
   }
}
