package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;

public class CoPPlanningTools
{
   public static int getCoPPointIndex(CoPPointName[] copPointList, CoPPointName copPointToSearch)
   {
      for (int counter = 0; counter < copPointList.length; counter ++)
         if (copPointList[counter] == copPointToSearch)
            return counter;
      return -1;
   }

   public static int getNumberOfSwingSegments(CoPPointName[] copPointList, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {
      int index1 = getCoPPointIndex(copPointList, entryCoPName);  // read as entryCoPIndex
      int index2 = getCoPPointIndex(copPointList, exitCoPName);   // read as exitCoPIndex
      return index1 < index2 ? index2 - index1 : copPointList.length + index2 - index1;
   }

   public static int getNumberOfTransferSegments(CoPPointName[] copPointList, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {
      int index1 = getCoPPointIndex(copPointList, entryCoPName);  // read as entryCoPIndex
      int index2 = getCoPPointIndex(copPointList, exitCoPName);   // read as exitCoPIndex
      return index2 < index1 ? index1 - index2 : copPointList.length + index1 - index2;
   }

   public static double getTransferStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {      
      int index1 = getCoPPointIndex(copPointList, entryCoPName);
      int index2 = getCoPPointIndex(copPointList, exitCoPName);
      return getTransferStateDuration(copPointList, segmentTimes, index1, index2);
   }
   
   public static double getTransferStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, int entryCoPIndex, int exitCoPIndex)
   {
      double duration = 0.0;
      for(int counter = exitCoPIndex + 1; counter != entryCoPIndex + 1; counter++)
      {
         if(counter == copPointList.length)
            counter = 0;
         duration += segmentTimes.get(copPointList[counter]);
      }
      return duration;
   }
   
   public static double getSwingStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {      
      int index1 = getCoPPointIndex(copPointList, entryCoPName);
      int index2 = getCoPPointIndex(copPointList, exitCoPName);
      return getSwingStateDuration(copPointList, segmentTimes, index1, index2);
   }
   
   public static double getSwingStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, int entryCoPIndex, int exitCoPIndex)
   {
      double duration = 0.0;
      for(int counter = entryCoPIndex + 1; counter != exitCoPIndex + 1; counter++)
      {
         if(counter == copPointList.length)
            counter = 0;
         duration += segmentTimes.get(copPointList[counter]);
      }
      return duration;
   }

   public static double getStepDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes)
   {
      double duration = 0.0;
      for (CoPPointName copPointName : copPointList)
         duration += segmentTimes.get(copPointName);
      return duration;
   }

   public static int getCoPPointIndex(List<CoPPointName> copPointList, CoPPointName copPointToSearch)
   {
      for (int counter = 0; counter < copPointList.size(); counter ++)
         if (copPointList.get(counter) == copPointToSearch)
            return counter;
      return -1;
   }
}

