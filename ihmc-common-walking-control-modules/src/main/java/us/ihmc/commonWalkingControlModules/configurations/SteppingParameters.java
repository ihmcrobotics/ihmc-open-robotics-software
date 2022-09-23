package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface SteppingParameters extends FootstepParameters
{
   public abstract double getMaxStepLength();

   default double getMaxBackwardStepLength()
   {
      return getMaxStepLength();
   }

   public abstract double getDefaultStepLength();

   public abstract double getMaxStepWidth();

   public abstract double getMinStepWidth();

   public abstract double getInPlaceWidth();

   public abstract double getMaxStepUp();

   public abstract double getMaxStepDown();

   public abstract double getMaxSwingHeightFromStanceFoot();

   /**
    * Returns the minimum swing height from the stance foot for this robot
    */
   public default double getMinSwingHeightFromStanceFoot()
   {
      return 0.1;
   }

   /**
    * Default swing height used by the controller. If a swing height is not specified or lies outside of the allowable range,
    * this value is used.
    */
   public default double getDefaultSwingHeightFromStanceFoot()
   {
      return getMinSwingHeightFromStanceFoot();
   }

   public default double getTurningStepWidth()
   {
      return 0.2;
   }

   /**
    * Custom waypoint positions are precomputed externally. During execution the initial foot pose might be different than expected,
    * and the preplanned trajectory might have the foot go backward before moving forward, for example. This provides a threshold
    * for the maximum angle from forward to use - lower is more restrictive, 90 deg max recommended.
    */
   public default double getCustomWaypointAngleThreshold()
   {
      return Math.toRadians(50.0);
   }

   /**
    * Returns the maximum angle the foot can turn outwards in a step.
    */
   public abstract double getMaxAngleTurnOutwards();

   /**
    * Returns the maximum angle the foot can turn inwards in a step.
    * <ul>
    * <li>zero indicates that the maximum inward rotation of the foot is with the foot pointing straight forward.
    * <li>a positive value indicates a limited range of motion such that the foot cannot rotate inward.
    * <li>a negative value indicates an extended range of motion such that the foot can point to the inside.
    * </ul>
    */
   public abstract double getMaxAngleTurnInwards();
}
