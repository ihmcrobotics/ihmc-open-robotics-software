package us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;

import java.util.List;

public class CustomPolicyTools
{
   static int getSegmentNumber(double time, List<? extends ContactStateProvider<?>> contactStateProviders)
   {
      double startTime = 0.0;
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         if (contactStateProviders.get(i).getTimeInterval().intervalContains(time - startTime))
            return i;

         startTime += contactStateProviders.get(i).getTimeInterval().getDuration();
      }

      return -1;
   }

   static double getTimeInSegment(int segmentNumber, double time, List<? extends ContactStateProvider<?>> contactStateProviders)
   {
      for (int i = 0; i < segmentNumber; i++)
         time -= contactStateProviders.get(i).getTimeInterval().getDuration();

      return time;
   }
}
