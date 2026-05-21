package us.ihmc.footstepPlanning;

import toolbox_msgs.AStarBodyPathPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * whether or not the planner checks for collisions.
    */
   default boolean getCheckForCollisions()
   {
      return get(checkForCollisions);
   }

   /**
    * Whether the body path plan is post-processed with the smoother.
    */
   default boolean getPerformSmoothing()
   {
      return get(performSmoothing);
   }

   /**
    * The node height of a vertex is determined as the average height of all cells
    * within this radius.
    */
   default double getSnapRadius()
   {
      return get(snapRadius);
   }

   /**
    * When computing the vertex height, cells that are this distance below the max
    * height are ignored when taking the average
    */
   default double getMinSnapHeightThreshold()
   {
      return get(minSnapHeightThreshold);
   }

   /**
    * This is the weight assigned to minimizing the inccline the path takes. The cost
    * is determined by the difference between the edge incline from the nominal
    * incline.
    */
   default double getInclineCostWeight()
   {
      return get(inclineCostWeight);
   }

   default double getObstacleClearanceWeight()
   {
      return get(obstacleClearanceWeight);
   }

   /**
    * This is a deadband applied to the incline in the search.
    */
   default double getInclineCostDeadband()
   {
      return get(inclineCostDeadband);
   }

   /**
    * The max incline, in degrees, that is allowed for the body path planner to
    * traverse.
    */
   default double getMaxIncline()
   {
      return get(maxIncline);
   }

   /**
    * Width of the collision box used for checking collisions with the environment.
    */
   default double getCollisionBoxSizeY()
   {
      return get(collisionBoxSizeY);
   }

   /**
    * Depth of the collision box used for checking collisions with the environment.
    */
   default double getCollisionBoxSizeX()
   {
      return get(collisionBoxSizeX);
   }

   /**
    * Intersection height below which collisions are ignored.
    */
   default double getCollisionBoxGroundClearance()
   {
      return get(collisionBoxGroundClearance);
   }

   /**
    * Weight placed on the gradient for avoiding collisions
    */
   default double getSmootherCollisionWeight()
   {
      return get(smootherCollisionWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the angle between successive
    * segments of the body path
    */
   default double getSmootherSmoothnessWeight()
   {
      return get(smootherSmoothnessWeight);
   }

   /**
    * Discount applied to the smoothness gradients of turn points.
    */
   default double getSmootherTurnPointSmoothnessDiscount()
   {
      return get(smootherTurnPointSmoothnessDiscount);
   }

   /**
    * Min curvature in degrees to penalize with a gradient.
    */
   default double getSmootherMinCurvatureToPenalize()
   {
      return get(smootherMinCurvatureToPenalize);
   }

   /**
    * Weight placed on the gradient for making the vertices of the body path plan an
    * equal distance apart.
    */
   default double getSmootherEqualSpacingWeight()
   {
      return get(smootherEqualSpacingWeight);
   }

   /**
    * Weight placed on a gradient that drives the waypoint towards the initial value
    */
   default double getSmootherDisplacementWeight()
   {
      return get(smootherDisplacementWeight);
   }

   /**
    * Gain applied to the smoother gradient for iterative modifications
    */
   default double getSmootherHillClimbGain()
   {
      return get(smootherHillClimbGain);
   }

   /**
    * Minimum gradient vector magnitude to terminate the smoother iterations
    */
   default double getSmootherGradientThresholdToTerminate()
   {
      return get(smootherGradientThresholdToTerminate);
   }

   /**
    * Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
    */
   default double getCollisionStartTolerance()
   {
      return get(collisionStartTolerance);
   }

   default AStarBodyPathPlannerParametersPacket getAsPacket()
   {
      AStarBodyPathPlannerParametersPacket packet = new AStarBodyPathPlannerParametersPacket();

      // Collision and surface normal settings
      packet.setCheckForCollisions(getCheckForCollisions());

      // Smoothing and roll cost parameters
      packet.setPerformSmoothing(getPerformSmoothing());

      // Snap settings
      packet.setSnapRadius(getSnapRadius());
      packet.setMinSnapHeightThreshold(getMinSnapHeightThreshold());

      // Incline parameters
      packet.setInclineCostWeight(getInclineCostWeight());
      packet.setInclineCostDeadband(getInclineCostDeadband());
      packet.setMaxIncline(getMaxIncline());

      packet.setObstacleClearanceCostWeight(getObstacleClearanceWeight());
      // Collision
      packet.setCollisionBoxSizeY(getCollisionBoxSizeY());
      packet.setCollisionBoxSizeX(getCollisionBoxSizeX());
      packet.setCollisionBoxGroundClearance(getCollisionBoxGroundClearance());
      packet.setCollisionStartTolerance(getCollisionStartTolerance());

      // Smoother parameters
      packet.setSmootherCollisionWeight(getSmootherCollisionWeight());
      packet.setSmootherSmoothnessWeight(getSmootherSmoothnessWeight());
      packet.setSmootherTurnPointSmoothnessDiscount(getSmootherTurnPointSmoothnessDiscount());
      packet.setSmootherMinCurvatureToPenalize(getSmootherMinCurvatureToPenalize());
      packet.setSmootherEqualSpacingWeight(getSmootherEqualSpacingWeight());
      packet.setSmootherDisplacementWeight(getSmootherDisplacementWeight());
      packet.setSmootherHillClimbGain(getSmootherHillClimbGain());
      packet.setSmootherGradientThresholdToTerminate(getSmootherGradientThresholdToTerminate());

      return packet;
   }
}
