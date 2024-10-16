package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.time.TimeIntervalTools;

import java.util.List;

/**
 * This is a tools class to validate contact sequences for the {@link CoMTrajectoryPlanner}.
 */
public class ContactStateProviderTools
{
   static double epsilonForContinuity = 1e-2;

   public static boolean checkContactSequenceIsValid(List<? extends ContactStateProvider> contactStateSequence)
   {
      if (!checkContactSequenceDoesNotEndInFlight(contactStateSequence))
         return false;

      return checkContactSequenceIsContinuous(contactStateSequence);
   }

   static boolean checkContactSequenceIsContinuous(List<? extends ContactStateProvider> contactStateSequence)
   {
      return TimeIntervalTools.isTimeSequenceContinuous(contactStateSequence, epsilonForContinuity);
   }

   static boolean checkContactSequenceDoesNotEndInFlight(List<? extends ContactStateProvider> contactStateSequence)
   {
      return contactStateSequence.get(contactStateSequence.size() - 1).getContactState().isLoadBearing();
   }

}
