package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.List;

/**
 * Parameters to tune the ICP (Instantaneous Capture Point) planner for each robot. The ICP planner
 * is a module that predicts the desired ICP trajectory accordingly to the upcoming footsteps. This
 * is one of the critical modules of the walking controller as the desired ICP is used to control
 * the balance of the robot.
 */
public interface CoPPlannerParameters
{
   /**
    * Refers to the number of CoP reference locations per support. Using two or more CoPs
    * is preferable as it helps the robot moving more towards the walking heading direction.
    * It also helps triggering the toe off earlier, and helps stepping down obstacles. However,
    * it requires to tune some more parameters.
    */
   int getNumberOfCoPWayPointsPerFoot();

   /**
    * <p>
    * This parameter indicates how far backward and forward in the foot the CoPs can be.
    * </p>
    * <p>
    * A large positive maximum value for the heel CoP can be helpful for backward steps.
    * The minimum value for the heel CoP should probably be 0.0, as a negative value would delay the toe off
    * when walking forward.
    * </p>
    */
   List<Vector2D> getCoPForwardOffsetBounds();

   /**
    * <p>
    * These parameters force the CoP locations inside or outside in the foot, and forward or backward.
    * </p>
    */
   List<Vector2D> getCoPOffsets();

   /**
    * This parameter is used when computing the CoP locations to make sure they are
    * contained inside a safe support polygon.
    */
   double getCoPSafeDistanceAwayFromSupportEdges();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * The forward offset of the ball CoP is computed according to the upcoming step length times the returned fraction.
    * </p>
    * <p>
    * One third seems to be a reasonable value.
    * </p>
    */
   double getStepLengthToBallCoPOffsetFactor();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * If true, and there are only two CoPs per foot, the ICP planner will put the ball CoP on the toes of the trailing foot.
    * </p>
    * <p>
    * If true, and there are three CoPs per foot, the ICP planner will put the toe CoP in this location, instead.
    * </p>
    * <p>
    * This is necessary when doing toe off in single-support.
    * </p>
    */
   boolean putExitCoPOnToes();


   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * If true, and there are only two CoPs per foot, the ICP planner will put the ball CoP on the toes of the trailing
    * foot when stepping down and forward.
    * </p>
    * <p>
    * If true, and there are three CoPs per foot, the ICP planner will put the toe CoP in this location, instead.
    * </p>
    */
   boolean useExitCoPOnToesForSteppingDown();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * Threshold used to figure out if the exit CoP should be put on the toes when stepping down.
    */
   double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * Threshold used to figure out if the exit CoP should be put on the toes when stepping down. An
    * absolute value is expected.
    */
   double getStepHeightThresholdForExitCoPOnToesWhenSteppingDown();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * If set to zero, the exit CoP will be put on the toes' edge when stepping down,
    * a positive value will put back the exit CoP towards the foot center.
    */
   double getCoPSafeDistanceAwayFromToesWhenSteppingDown();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * Sets the distance inside from the front edge the exit CoP is located when putting the exit CoP on the toes.
    * </p>
    * <p>
    * If using two CoPs, this refers to the ball CoP location. If using three, this is the toe CoP location.
    * </p>
    */
   double getExitCoPForwardSafetyMarginOnToes();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * Threshold used to figure out if the exit CoP should be put on the toes.
    */
   double getStepLengthThresholdForExitCoPOnToes();
}