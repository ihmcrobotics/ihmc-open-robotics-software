package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class CoPPlanningToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testArrayLinearSearch()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP, CoPPointName.BALL_COP, CoPPointName.TOE_COP, CoPPointName.TOE_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.BALL_COP) == 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testArrayLinearSearchMissingPoint()
   {
      CoPPointName[] list = new CoPPointName[]{CoPPointName.HEEL_COP, CoPPointName.BALL_COP, CoPPointName.TOE_COP, CoPPointName.TOE_COP};
      assertTrue(CoPPlanningTools.getCoPPointIndex(list, CoPPointName.MIDFEET_COP) == -1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
