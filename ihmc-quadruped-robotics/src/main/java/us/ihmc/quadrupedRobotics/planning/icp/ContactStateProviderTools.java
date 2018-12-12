package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;
import us.ihmc.quadrupedRobotics.planning.ContactState;

import java.util.List;

public class ContactStateProviderTools
{
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
         if (!MathTools.epsilonEquals(contactStateSequence.get(index).getTimeInterval().getEndTime(), contactStateSequence.get(index + 1).getTimeInterval().getStartTime(), 1e-4))
            return false;
      }

      return true;
   }

   static boolean checkContactSequenceDoesNotEndInFlight(List<? extends ContactStateProvider> contactStateSequence)
   {
      return contactStateSequence.get(contactStateSequence.size() - 1).getContactState().isLoadBearing();
   }

}
