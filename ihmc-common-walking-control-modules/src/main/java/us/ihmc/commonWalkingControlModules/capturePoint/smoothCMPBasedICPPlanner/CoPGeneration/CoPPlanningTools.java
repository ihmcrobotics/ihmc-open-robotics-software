package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;

import java.util.List;

public class CoPPlanningTools
{
   public static int getCoPPointIndex(CoPPointName[] copPointList, CoPPointName copPointToSearch)
   {
      for (int counter = 0; counter < copPointList.length; counter ++)
         if (copPointList[counter] == copPointToSearch)
            return counter;
      return -1;
   }

   public static int getCoPPointIndex(List<CoPPointName> copPointList, CoPPointName copPointToSearch)
   {
      for (int counter = 0; counter < copPointList.size(); counter ++)
         if (copPointList.get(counter) == copPointToSearch)
            return counter;
      return -1; 
   }
}

