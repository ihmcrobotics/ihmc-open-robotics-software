package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;

public class CoPPlanningTools
{
   private static int tempIndex1, tempIndex2, counter;
   private static double tempDouble;
   
   public static int getCoPPointIndex(CoPPointName[] copPointList, CoPPointName copPointToSearch)
   {
      for (counter = 0; counter < copPointList.length; counter ++)
         if (copPointList[counter] == copPointToSearch)
            return counter;
      return -1;
   }

   public static int getNumberOfSwingSegments(CoPPointName[] copPointList, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {
      tempIndex1 = getCoPPointIndex(copPointList, entryCoPName);  // read as entryCoPIndex
      tempIndex2 = getCoPPointIndex(copPointList, exitCoPName);   // read as exitCoPIndex
      return tempIndex1 < tempIndex2 ? tempIndex2 - tempIndex1 : copPointList.length + tempIndex2 - tempIndex1;
   }

   public static int getNumberOfTransferSegments(CoPPointName[] copPointList, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {
      tempIndex1 = getCoPPointIndex(copPointList, entryCoPName);  // read as entryCoPIndex
      tempIndex2 = getCoPPointIndex(copPointList, exitCoPName);   // read as exitCoPIndex
      return tempIndex2 < tempIndex1 ? tempIndex1 - tempIndex2 : copPointList.length + tempIndex1 - tempIndex2;
   }

   public static double getTransferStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {      
      tempIndex1 = getCoPPointIndex(copPointList, entryCoPName); 
      tempIndex2 = getCoPPointIndex(copPointList, exitCoPName);
      return getTransferStateDuration(copPointList, segmentTimes, tempIndex1, tempIndex2);
   }
   
   public static double getTransferStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, int entryCoPIndex, int exitCoPIndex)
   {
      tempDouble = 0.0;
      for(counter = exitCoPIndex + 1; counter != entryCoPIndex + 1; counter++)
      {
         if(counter == copPointList.length)
            counter = 0;
         tempDouble += segmentTimes.get(copPointList[counter]);
      }
      return tempDouble;
   }
   
   public static double getSwingStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, CoPPointName entryCoPName, CoPPointName exitCoPName)
   {      
      tempIndex1 = getCoPPointIndex(copPointList, entryCoPName); 
      tempIndex2 = getCoPPointIndex(copPointList, exitCoPName);
      return getSwingStateDuration(copPointList, segmentTimes, tempIndex1, tempIndex2);
   }
   
   public static double getSwingStateDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes, int entryCoPIndex, int exitCoPIndex)
   {
      tempDouble = 0.0;
      for(counter = entryCoPIndex + 1; counter != exitCoPIndex + 1; counter++)
      {
         if(counter == copPointList.length)
            counter = 0;
         tempDouble += segmentTimes.get(copPointList[counter]);
      }
      return tempDouble;
   }

   public static double getStepDuration(CoPPointName[] copPointList, EnumMap<CoPPointName, Double> segmentTimes)
   {
      tempDouble = 0.0;
      for (counter = 0; counter < copPointList.length; counter++)
         tempDouble += segmentTimes.get(copPointList[counter]);
      return tempDouble;
   }

   public static int getCoPPointIndex(List<CoPPointName> copPointList, CoPPointName copPointToSearch)
   {
      for (counter = 0; counter < copPointList.size(); counter ++)
         if (copPointList.get(counter) == copPointToSearch)
            return counter;
      return -1;
   }
}

