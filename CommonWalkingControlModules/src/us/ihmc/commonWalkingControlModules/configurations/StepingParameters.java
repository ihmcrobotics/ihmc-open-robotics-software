package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface StepingParameters extends FootstepParameters
{
   public abstract double getMaxStepLength();

   public abstract double getDefaultStepLength();

   public abstract double getMaxStepWidth();

   public abstract double getMinStepWidth();

   public abstract double getInPlaceWidth();

   public abstract double getDesiredStepForward(); //???

   public abstract double getStepPitch(); //???

   public abstract double getMaxStepUp();

   public abstract double getMaxStepDown();

   public abstract double getMaxAngleTurnOutwards(); //the maximum angle the foot can turn outwards in a step

   public abstract double getMaxAngleTurnInwards(); //the maximum angle the foot can turn inwards in a step
}
