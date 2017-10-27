package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;

public class CoPPlanningToolsTest
{
   @Test(timeout = 30000)
   public void testArrayLinearSearch()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP, CoPPointName.BALL_COP, CoPPointName.TOE_COP, CoPPointName.TOE_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.BALL_COP) == 2);
   }
   
   @Test(timeout = 30000)
   public void testArrayLinearSearchMissingPoint()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.HEEL_COP, CoPPointName.BALL_COP, CoPPointName.TOE_COP, CoPPointName.TOE_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.MIDFEET_COP) == -1);
   }

   @Test(timeout = 30000)
   public void testArrayListLinearSearch()
   {
      ArrayList<CoPPointName> list = new ArrayList<>();
      list.add(CoPPointName.MIDFEET_COP);
      list.add(CoPPointName.HEEL_COP);
      list.add(CoPPointName.BALL_COP);
      list.add(CoPPointName.TOE_COP);
      list.add(CoPPointName.TOE_COP);
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.HEEL_COP) == 1);
   }

   @Test(timeout = 30000)
   public void testArrayListLinearSearchMissingPoint()
   {
      ArrayList<CoPPointName> list = new ArrayList<>();
      list.add(CoPPointName.MIDFEET_COP);
      list.add(CoPPointName.HEEL_COP);
      list.add(CoPPointName.BALL_COP);
      list.add(CoPPointName.BALL_COP);
      list.add(CoPPointName.MIDFEET_COP);
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.TOE_COP) == -1);
   }

}
