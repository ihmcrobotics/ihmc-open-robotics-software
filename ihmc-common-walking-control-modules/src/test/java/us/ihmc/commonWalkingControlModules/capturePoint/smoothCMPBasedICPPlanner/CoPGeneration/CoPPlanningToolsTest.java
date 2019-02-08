package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class CoPPlanningToolsTest
{
   @Test
   public void testArrayLinearSearch()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.ENTRY_COP, CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP, CoPPointName.EXIT_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.MIDFOOT_COP) == 2);
   }

   @Test
   public void testArrayLinearSearchMissingPoint()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.ENTRY_COP, CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP, CoPPointName.EXIT_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.MIDFEET_COP) == -1);
   }

   @Test
   public void testArrayListLinearSearch()
   {
      ArrayList<CoPPointName> list = new ArrayList<>();
      list.add(CoPPointName.MIDFEET_COP);
      list.add(CoPPointName.ENTRY_COP);
      list.add(CoPPointName.MIDFOOT_COP);
      list.add(CoPPointName.EXIT_COP);
      list.add(CoPPointName.EXIT_COP);
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.ENTRY_COP) == 1);
   }

   @Test
   public void testArrayListLinearSearchMissingPoint()
   {
      ArrayList<CoPPointName> list = new ArrayList<>();
      list.add(CoPPointName.MIDFEET_COP);
      list.add(CoPPointName.ENTRY_COP);
      list.add(CoPPointName.MIDFOOT_COP);
      list.add(CoPPointName.MIDFOOT_COP);
      list.add(CoPPointName.MIDFEET_COP);
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.EXIT_COP) == -1);
   }

}
