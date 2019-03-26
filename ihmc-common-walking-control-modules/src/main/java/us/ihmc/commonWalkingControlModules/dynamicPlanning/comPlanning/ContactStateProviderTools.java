package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;

import java.util.List;

/**
 * This is a tools class to validate contact sequences for the {@link CoMTrajectoryPlanner}.
 */
public class ContactStateProviderTools
{
   static double epsilonForContinuity = 1e-4;

   static boolean checkContactSequenceIsValid(List<? extends ContactStateProvider> contactStateSequence)
   {
      if (!checkContactSequenceDoesNotEndInFlight(contactStateSequence))
         return false;

      return checkContactSequenceIsContinuous(contactStateSequence);
   }

   static boolean checkContactSequenceIsContinuous(List<? extends ContactStateProvider> contactStateSequence)
   {
      for (int index = 0; index < contactStateSequence.size() - 1; index++)
      {
         if (!MathTools.epsilonEquals(contactStateSequence.get(index).getTimeInterval().getEndTime(),
                                      contactStateSequence.get(index + 1).getTimeInterval().getStartTime(), epsilonForContinuity))
            return false;
      }

      return true;
   }

   static boolean checkContactSequenceDoesNotEndInFlight(List<? extends ContactStateProvider> contactStateSequence)
   {
      return contactStateSequence.get(contactStateSequence.size() - 1).getContactState().isLoadBearing();
   }

}
