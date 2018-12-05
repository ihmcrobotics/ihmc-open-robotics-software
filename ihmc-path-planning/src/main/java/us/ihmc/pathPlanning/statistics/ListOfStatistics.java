package us.ihmc.pathPlanning.statistics;

import java.util.ArrayList;
import java.util.List;

public class ListOfStatistics implements PlannerStatistics<ListOfStatistics>
{
   private final List<PlannerStatistics<?>> statisticsList = new ArrayList<>();

   @Override
   public StatisticsType getStatisticsType()
   {
      return StatisticsType.LIST;
   }

   @Override
   public void set(ListOfStatistics listOfStatistics)
   {
      clear();
      addStatisticsList(listOfStatistics);
   }

   public void addStatistics(PlannerStatistics<?> statistics)
   {
      if (statistics != null)
      {
         if (statistics instanceof ListOfStatistics)
            addStatisticsList((ListOfStatistics) statistics);
         else
            statisticsList.add(statistics);
      }
   }

   public void addStatisticsList(ListOfStatistics listOfStatistics)
   {
      if (listOfStatistics == null)
         return;

      for (int i = 0; i < listOfStatistics.getNumberOfStatistics(); i++)
         addStatistics(listOfStatistics.getStatistics(i));
   }

   public PlannerStatistics<?> getStatistics(int statisticsIndex)
   {
      return statisticsList.get(statisticsIndex);
   }

   public void clear()
   {
      statisticsList.clear();
   }

   public int getNumberOfStatistics()
   {
      return statisticsList.size();
   }

   public PlannerStatistics<?> pollStatistics()
   {
      if (statisticsList.isEmpty())
         return null;
      else
         return statisticsList.remove(getNumberOfStatistics() - 1);
   }

}
