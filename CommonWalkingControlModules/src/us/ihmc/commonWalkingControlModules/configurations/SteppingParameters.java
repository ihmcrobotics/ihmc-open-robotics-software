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

   public abstract double getDesiredStepForward(); //???

   public abstract double getStepPitch(); //???

   public abstract double getMaxStepUp();

   public abstract double getMaxStepDown();

   public abstract double getMaxSwingHeightFromStanceFoot();
   
   public abstract double  getMinSwingHeightFromStanceFoot();
  
   public abstract double getMaxAngleTurnOutwards(); //the maximum angle the foot can turn outwards in a step

   public abstract double getMaxAngleTurnInwards(); //the maximum angle the foot can turn inwards in a step

   public abstract double getMinAreaPercentForValidFootstep(); //the minimum percentage of area that the robot can take a step with

   public abstract double getDangerAreaPercentForValidFootstep(); //the percentage of footstep area below which large footsteps cannot be taken
}
