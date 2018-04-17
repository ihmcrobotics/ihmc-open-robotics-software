package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.EnumMap;
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
    * Name of the cop point that is designed as the entry, which is the first CoP for the swing phase. Typically Heel.
    */
   CoPPointName getEntryCoPName();

   /**
    * Name of the cop point that is designed as the exit, which is the last CoP for the swing phase. Typically Toe.
    */
   CoPPointName getExitCoPName();

   /**
    * <p>
    * This parameter indicates how far backward and forward in the foot the CoPs can be.
    * </p>
    * <p>
    * A large positive maximum value for the entry CoP can be helpful for backward steps.
    * The minimum value for the entry CoP should probably be 0.0, as a negative value would delay the toe off
    * when walking forward.
    * </p>
    * <p>
    * A large positive maximum value for the exit CoP here can improve significantly long
    * forward steps and help trigger the exit off earlier.
    * The minimum value for the exit CoP indicates how far back in the foot it can be. For instance,
    * -0.02m will let the robot move slightly backward in single support when doing back steps.
    * </p>
    * <p>
    * The data is organized such that the entry CoP bounds are the first entry in the list, and
    * the exit CoP bounds are the second entry in the list. The minimum value is then the X field
    * in the vector, while the maximum value is the Y field in the vector.
    * </p>
    */
   EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot();

   /**
    * <p>
    * These parameters force the CoP locations inside or outside in the foot, and forward or backward.
    * </p>
    * <p>
    * The values for the exit CoP are only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * The offsets themselves are in the foot frame. The X offset is the forward offset in the foot,
    * while the Y offset is the inside offset.
    * </p>
    */
   EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame();

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
    * The forward offset of the exit CoP is computed according to the upcoming step length times the returned fraction.
    * </p>
    * <p>
    * One third seems to be a reasonable value.
    * </p>
    */
   EnumMap<CoPPointName, Double> getStepLengthToCoPOffsetFactors();

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