package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface SteppingParameters extends FootstepParameters
{
   default double getMaxStepLength()
   {
      return 0.7;
   }

   default double getMaxBackwardStepLength()
   {
      return getMaxStepLength();
   }

   default double getDefaultStepLength()
   {
      return 0.4;
   }

   default double getMaxStepWidth()
   {
      return 0.8;
   }

   default double getMinStepWidth()
   {
      return 0.12;
   }

   default double getInPlaceWidth()
   {
      return 0.22;
   }

   default double getMaxStepUp()
   {
      return 0.25;
   }

   default double getMaxStepDown()
   {
      return 0.2;
   }

   default double getTurningStepWidth()
   {
      return 0.2;
   }

   /**
    * Returns the maximum angle the foot can turn outwards in a step.
    */
   default double getMaxAngleTurnOutwards()
   {
      return 0.65;
   }

   /**
    * Returns the maximum angle the foot can turn inwards in a step.
    * <ul>
    * <li>zero indicates that the maximum inward rotation of the foot is with the foot pointing straight forward.
    * <li>a positive value indicates a limited range of motion such that the foot cannot rotate inward.
    * <li>a negative value indicates an extended range of motion such that the foot can point to the inside.
    * </ul>
    */
   default double getMaxAngleTurnInwards()
   {
      return 0.0;
   }
}
