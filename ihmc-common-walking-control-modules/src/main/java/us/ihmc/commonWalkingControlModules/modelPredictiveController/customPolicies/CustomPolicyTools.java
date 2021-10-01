package us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

import java.util.List;

public class CustomPolicyTools
{
   static int getSegmentNumber(double time, List<? extends TimeIntervalReadOnly> contactStateProviders)
   {
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         if (contactStateProviders.get(i).intervalContains(time))
            return i;
      }

      return -1;
   }

   static double getTimeInSegment(int segmentNumber, double time, List<? extends TimeIntervalReadOnly> contactStateProviders)
   {
      return time - contactStateProviders.get(segmentNumber).getStartTime();
   }
}
