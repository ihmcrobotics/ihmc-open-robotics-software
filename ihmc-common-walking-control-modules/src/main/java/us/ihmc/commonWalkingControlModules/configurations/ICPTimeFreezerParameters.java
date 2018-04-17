package us.ihmc.commonWalkingControlModules.configurations;

public interface ICPTimeFreezerParameters
{
   /**
    * Slow down factor on the time when doing partial time freeze
    */
   default double getFreezeTimeFactor()
   {
      return 0.9;
   }

   /**
    * Threshold on the ICP error used to trigger the complete time freeze.
    */
   double getMaxInstantaneousCapturePointErrorForStartingSwing();

   /**
    * Threshold on the ICP error used to trigger the partial time freeze which consists in slowing
    * down time.
    */
   double getMaxAllowedErrorWithoutPartialTimeFreeze();

   /**
    * Enable / disable time freezing. When enabled, the time freezer will first slow down time when
    * the actual ICP is behind the plan, and totally freeze time if the actual ICP is far behind the
    * plan. In summary, this makes the plan wait for the actual ICP when there is a lag.
    */
   boolean getDoTimeFreezing();

}