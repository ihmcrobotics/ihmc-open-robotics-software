package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface SteppingParameters extends FootstepParameters
{
   public abstract double getMaxStepLength();

   public abstract double getDefaultStepLength();

   public abstract double getMaxStepWidth();

   public abstract double getMinStepWidth();

   public abstract double getInPlaceWidth();

   public abstract double getDesiredStepForward();

   public abstract double getStepPitch();

   public abstract double getMaxStepUp();

   public abstract double getMaxStepDown();

   public abstract double getMaxSwingHeightFromStanceFoot();

   /**
    * Returns the minimum swing height from the stance foot for this robot. It is also the default swing height
    * used in the controller unless a different value is specified.
    */
   public default double getMinSwingHeightFromStanceFoot()
   {
      return 0.1;
   }

   /**
    * Returns the maximum angle the foot can turn outwards in a step.
    */
   public abstract double getMaxAngleTurnOutwards();

   /**
    * Returns the maximum angle the foot can turn inwards in a step.
    */
   public abstract double getMaxAngleTurnInwards();

   /**
    * Returns the minimum percentage of area that the robot can take a step with.
    */
   public abstract double getMinAreaPercentForValidFootstep();

   /**
    * Returns the percentage of footstep area below which large footsteps cannot be taken.
    */
   public abstract double getDangerAreaPercentForValidFootstep();
}
