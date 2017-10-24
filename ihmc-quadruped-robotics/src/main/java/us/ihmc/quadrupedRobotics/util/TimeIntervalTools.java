package us.ihmc.quadrupedRobotics.util;

import java.util.Comparator;
import java.util.List;

import us.ihmc.tools.lists.ListSorter;

@SuppressWarnings("unchecked")
public class TimeIntervalTools
{
   static public void sortByStartTime(PreallocatedList<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      PreallocatedListSorter.sort((PreallocatedList<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByStartTime);
   }

   static public void sortByReverseStartTime(PreallocatedList<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      PreallocatedListSorter.sort((PreallocatedList<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByStartTime.reversed());
   }

   static public void sortByEndTime(PreallocatedList<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      PreallocatedListSorter.sort((PreallocatedList<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByEndTime);
   }

   static public void sortByReverseEndTime(PreallocatedList<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      PreallocatedListSorter.sort((PreallocatedList<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByEndTime.reversed());
   }

   static public void sortByStartTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      ListSorter.sort((List<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByStartTime);
   }

   static public void sortByReverseStartTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      ListSorter.sort((List<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByStartTime.reversed());
   }

   static public void sortByEndTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      ListSorter.sort((List<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByEndTime);
   }

   static public void sortByReverseEndTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      ListSorter.sort((List<TimeIntervalProvider>)timeIntervalProviders, compareTimeIntervalProvidersByEndTime.reversed());
   }

   static public void removeStartTimesLessThan(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getStartTime() < time)
         {
            timeIntervalProviders.remove(i);
            i--;
         }
      }
   }

   static public void removeStartTimesLessThanOrEqualTo(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getStartTime() <= time)
         {
            timeIntervalProviders.remove(i);
            i--;
         }
      }
   }

   static public void removeStartTimesGreaterThan(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = timeIntervalProviders.size() - 1; i >= 0; i--)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getStartTime() > time)
         {
            timeIntervalProviders.remove(i);
         }
      }
   }

   static public void removeStartTimesGreaterThanOrEqualTo(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = timeIntervalProviders.size() - 1; i >= 0; i--)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getStartTime() >= time)
         {
            timeIntervalProviders.remove(i);
         }
      }
   }

   static public void removeEndTimesLessThan(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getEndTime() < time)
         {
            timeIntervalProviders.remove(i);
            i--;
         }
      }
   }

   static public void removeEndTimesLessThanOrEqualTo(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getEndTime() <= time)
         {
            timeIntervalProviders.remove(i);
            i--;
         }
      }
   }

   static public void removeEndTimesGreaterThan(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = timeIntervalProviders.size() - 1; i >= 0; i--)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getEndTime() > time)
         {
            timeIntervalProviders.remove(i);
         }
      }
   }

   static public void removeEndTimesGreaterThanOrEqualTo(double time, List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      for (int i = timeIntervalProviders.size() - 1; i >= 0; i--)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getEndTime() >= time)
         {
            timeIntervalProviders.remove(i);
         }
      }
   }

   static private Comparator<TimeIntervalProvider> compareTimeIntervalProvidersByStartTime = new Comparator<TimeIntervalProvider>()
   {
      @Override
      public int compare(TimeIntervalProvider a, TimeIntervalProvider b)
      {
         double startTimeA = a.getTimeInterval().getStartTime();
         double startTimeB = b.getTimeInterval().getStartTime();
         return Double.compare(startTimeA, startTimeB);
      }
   };

   static private Comparator<TimeIntervalProvider> compareTimeIntervalProvidersByEndTime = new Comparator<TimeIntervalProvider>()
   {
      @Override
      public int compare(TimeIntervalProvider a, TimeIntervalProvider b)
      {
         double endTimeA = a.getTimeInterval().getEndTime();
         double endTimeB = b.getTimeInterval().getEndTime();
         return Double.compare(endTimeA, endTimeB);
      }
   };
}
