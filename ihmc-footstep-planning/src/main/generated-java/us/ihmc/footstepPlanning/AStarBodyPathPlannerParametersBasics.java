package us.ihmc.footstepPlanning;

import static us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters.collisionStartTolerance;

import toolbox_msgs.AStarBodyPathPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersBasics extends AStarBodyPathPlannerParametersReadOnly, StoredPropertySetBasics
{
   /**
    * whether or not the planner checks for collisions.
    */
   default void setCheckForCollisions(boolean checkForCollisions)
   {
      set(AStarBodyPathPlannerParameters.checkForCollisions, checkForCollisions);
   }

   /**
    * Whether the body path plan is post-processed with the smoother.
    */
   default void setPerformSmoothing(boolean performSmoothing)
   {
      set(AStarBodyPathPlannerParameters.performSmoothing, performSmoothing);
   }

   /**
    * The node height of a vertex is determined as the average height of all cells
    * within this radius.
    */
   default void setSnapRadius(double snapRadius)
   {
      set(AStarBodyPathPlannerParameters.snapRadius, snapRadius);
   }

   /**
    * When computing the vertex height, cells that are this distance below the max
    * height are ignored when taking the average
    */
   default void setMinSnapHeightThreshold(double minSnapHeightThreshold)
   {
      set(AStarBodyPathPlannerParameters.minSnapHeightThreshold, minSnapHeightThreshold);
   }

   /**
    * This is the weight assigned to minimizing the inccline the path takes. The cost
    * is determined by the difference between the edge incline from the nominal
    * incline.
    */
   default void setInclineCostWeight(double inclineCostWeight)
   {
      set(AStarBodyPathPlannerParameters.inclineCostWeight, inclineCostWeight);
   }

   /**
    * This is a deadband applied to the incline in the search.
    */
   default void setInclineCostDeadband(double inclineCostDeadband)
   {
      set(AStarBodyPathPlannerParameters.inclineCostDeadband, inclineCostDeadband);
   }

   /**
    * The max incline, in degrees, that is allowed for the body path planner to
    * traverse.
    */
   default void setMaxIncline(double maxIncline)
   {
      set(AStarBodyPathPlannerParameters.maxIncline, maxIncline);
   }

   /**
    * Width of the collision box used for checking collisions with the environment.
    */
   default void setCollisionBoxSizeY(double collisionBoxSizeY)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxSizeY, collisionBoxSizeY);
   }

   /**
    * Depth of the collision box used for checking collisions with the environment.
    */
   default void setCollisionBoxSizeX(double collisionBoxSizeX)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxSizeX, collisionBoxSizeX);
   }

   /**
    * Intersection height below which collisions are ignored.
    */
   default void setCollisionBoxGroundClearance(double collisionBoxGroundClearance)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxGroundClearance, collisionBoxGroundClearance);
   }

   /**
    * Weight placed on the gradient for avoiding collisions
    */
   default void setSmootherCollisionWeight(double smootherCollisionWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherCollisionWeight, smootherCollisionWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the angle between successive
    * segments of the body path
    */
   default void setSmootherSmoothnessWeight(double smootherSmoothnessWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherSmoothnessWeight, smootherSmoothnessWeight);
   }

   /**
    * Discount applied to the smoothness gradients of turn points.
    */
   default void setSmootherTurnPointSmoothnessDiscount(double smootherTurnPointSmoothnessDiscount)
   {
      set(AStarBodyPathPlannerParameters.smootherTurnPointSmoothnessDiscount, smootherTurnPointSmoothnessDiscount);
   }

   /**
    * Min curvature in degrees to penalize with a gradient.
    */
   default void setSmootherMinCurvatureToPenalize(double smootherMinCurvatureToPenalize)
   {
      set(AStarBodyPathPlannerParameters.smootherMinCurvatureToPenalize, smootherMinCurvatureToPenalize);
   }

   /**
    * Weight placed on the gradient for making the vertices of the body path plan an
    * equal distance apart.
    */
   default void setSmootherEqualSpacingWeight(double smootherEqualSpacingWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherEqualSpacingWeight, smootherEqualSpacingWeight);
   }

   /**
    * Weight placed on a gradient that drives the waypoint towards the initial value
    */
   default void setSmootherDisplacementWeight(double smootherDisplacementWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherDisplacementWeight, smootherDisplacementWeight);
   }

   /**
    * Gain applied to the smoother gradient for iterative modifications
    */
   default void setSmootherHillClimbGain(double smootherHillClimbGain)
   {
      set(AStarBodyPathPlannerParameters.smootherHillClimbGain, smootherHillClimbGain);
   }

   /**
    * Minimum gradient vector magnitude to terminate the smoother iterations
    */
   default void setSmootherGradientThresholdToTerminate(double smootherGradientThresholdToTerminate)
   {
      set(AStarBodyPathPlannerParameters.smootherGradientThresholdToTerminate, smootherGradientThresholdToTerminate);
   }

   /**
    * Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
    */
   default void setCollisionStartTolerance(double collisionStartTolerance)
   {
      set(AStarBodyPathPlannerParameters.collisionStartTolerance, collisionStartTolerance);
   }

   default void set(AStarBodyPathPlannerParametersPacket packet)
   {
      double noValue = AStarBodyPathPlannerParametersPacket.DEFAULT_NO_VALUE;

      setCheckForCollisions(packet.getCheckForCollisions());
      setPerformSmoothing(packet.getPerformSmoothing());

      if (packet.getSnapRadius() != noValue)
         setSnapRadius(packet.getSnapRadius());
      if (packet.getMinSnapHeightThreshold() != noValue)
         setMinSnapHeightThreshold(packet.getMinSnapHeightThreshold());
      if (packet.getInclineCostWeight() != noValue)
         setInclineCostWeight(packet.getInclineCostWeight());
      if (packet.getInclineCostDeadband() != noValue)
         setInclineCostDeadband(packet.getInclineCostDeadband());
      if (packet.getMaxIncline() != noValue)
         setMaxIncline(packet.getMaxIncline());
      if (packet.getCollisionBoxSizeY() != noValue)
         setCollisionBoxSizeY(packet.getCollisionBoxSizeY());
      if (packet.getCollisionBoxSizeX() != noValue)
         setCollisionBoxSizeX(packet.getCollisionBoxSizeX());
      if (packet.getCollisionBoxGroundClearance() != noValue)
         setCollisionBoxGroundClearance(packet.getCollisionBoxGroundClearance());
      if (packet.getSmootherCollisionWeight() != noValue)
         setSmootherCollisionWeight(packet.getSmootherCollisionWeight());
      if (packet.getSmootherSmoothnessWeight() != noValue)
         setSmootherSmoothnessWeight(packet.getSmootherSmoothnessWeight());
      if (packet.getSmootherTurnPointSmoothnessDiscount() != noValue)
         setSmootherTurnPointSmoothnessDiscount(packet.getSmootherTurnPointSmoothnessDiscount());
      if (packet.getSmootherMinCurvatureToPenalize() != noValue)
         setSmootherMinCurvatureToPenalize(packet.getSmootherMinCurvatureToPenalize());
      if (packet.getSmootherEqualSpacingWeight() != noValue)
         setSmootherEqualSpacingWeight(packet.getSmootherEqualSpacingWeight());
      if (packet.getSmootherDisplacementWeight() != noValue)
         setSmootherDisplacementWeight(packet.getSmootherDisplacementWeight());
      if (packet.getSmootherHillClimbGain() != noValue)
         setSmootherHillClimbGain(packet.getSmootherHillClimbGain());
      if (packet.getSmootherGradientThresholdToTerminate() != noValue)
         setSmootherGradientThresholdToTerminate(packet.getSmootherGradientThresholdToTerminate());
      if (packet.getCollisionStartTolerance() != noValue)
         setCollisionStartTolerance(packet.getCollisionStartTolerance());
   }
}
