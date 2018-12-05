package us.ihmc.pathPlanning.statistics;

public interface PlannerStatistics<T extends PlannerStatistics<T>>
{
   StatisticsType getStatisticsType();

   void set(T other);
}
