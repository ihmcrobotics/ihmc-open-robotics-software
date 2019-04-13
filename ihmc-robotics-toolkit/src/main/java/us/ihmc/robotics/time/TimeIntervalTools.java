package us.ihmc.robotics.time;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@SuppressWarnings("unchecked")
public class TimeIntervalTools
{
   public static boolean doIntervalsOverlap(TimeIntervalReadOnly intervalA, TimeIntervalReadOnly intervalB)
   {
      if (intervalA.intervalContains(intervalB.getStartTime()))
         return true;

      if (intervalA.intervalContains(intervalB.getEndTime()))
         return true;

      if (intervalB.intervalContains(intervalA.getStartTime()))
         return true;

      return intervalB.intervalContains(intervalB.getEndTime());
   }

   static public void sortByStartTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      sort((List<TimeIntervalProvider>) timeIntervalProviders, startTimeComparator);
   }

   static public void sortByReverseStartTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      sort((List<TimeIntervalProvider>) timeIntervalProviders, startTimeComparator.reversed());
   }

   static public void sortByEndTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      sort((List<TimeIntervalProvider>) timeIntervalProviders, endTimeComparator);
   }

   static public void sortByReverseEndTime(List<? extends TimeIntervalProvider> timeIntervalProviders)
   {
      sort((List<TimeIntervalProvider>) timeIntervalProviders, endTimeComparator.reversed());
   }

   private static <T> void sort(List<T> ts, Comparator<T> comparator)
   {
      boolean ordered = false;

      while (!ordered)
      {
         ordered = true;
         for (int i = 0; i < ts.size() - 1; i++)
         {
            T a = ts.get(i);
            T b = ts.get(i + 1);

            if (comparator.compare(a, b) > 0)
            {
               ordered = false;
               swap(ts, i, i + 1);
            }
         }
      }
   }

   private static <T> void swap(List<T> ts, int a, int b)
   {
      T tmp = ts.get(a);
      ts.set(a, ts.get(b));
      ts.set(b, tmp);
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

   static public <T extends TimeIntervalProvider> List<T> removeAndReturnEndTimesLessThan(double time, List<T> timeIntervalProviders)
   {
      List<T> timeIntervalProvidersToRemove = new ArrayList<>();

      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().getEndTime() < time)
         {
            timeIntervalProvidersToRemove.add(timeIntervalProviders.get(i));
         }
      }

      return timeIntervalProvidersToRemove;
   }

   static public <T extends TimeIntervalProvider> List<T> getIntervalsContainingTime(double time, List<T> timeIntervalProviders)
   {
      List<T> timeIntervalProvidersToRemove = new ArrayList<>();

      for (int i = 0; i < timeIntervalProviders.size(); i++)
      {
         if (timeIntervalProviders.get(i).getTimeInterval().intervalContains(time))
         {
            timeIntervalProvidersToRemove.add(timeIntervalProviders.get(i));
         }
      }

      return timeIntervalProvidersToRemove;
   }

   public static Comparator<TimeIntervalProvider> startTimeComparator = (TimeIntervalProvider a, TimeIntervalProvider b) -> {
      double startTimeA = a.getTimeInterval().getStartTime();
      double startTimeB = b.getTimeInterval().getStartTime();
      return Double.compare(startTimeA, startTimeB);
   };

   public static Comparator<TimeIntervalProvider> endTimeComparator = (TimeIntervalProvider a, TimeIntervalProvider b) -> {
      double endTimeA = a.getTimeInterval().getEndTime();
      double endTimeB = b.getTimeInterval().getEndTime();
      return Double.compare(endTimeA, endTimeB);
   };
}
