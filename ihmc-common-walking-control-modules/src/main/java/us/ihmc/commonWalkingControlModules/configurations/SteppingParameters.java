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

   public default double getTurningStepWidth()
   {
      return 0.2;
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
