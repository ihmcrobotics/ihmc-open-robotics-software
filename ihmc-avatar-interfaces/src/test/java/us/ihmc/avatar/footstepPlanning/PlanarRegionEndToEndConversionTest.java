package us.ihmc.avatar.footstepPlanning;

import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class PlanarRegionEndToEndConversionTest
{
   @Test
   public void testDataSets()
   {
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(buildFilter(getTestableFilter()));

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();
      RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(PlanarRegion::new);

      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
         PlanarRegionsListMessage listMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         listCommand.setFromMessage(listMessage);

         planarRegions.clear();
         for (int regionIdx = 0; regionIdx < listCommand.getNumberOfPlanarRegions(); regionIdx++)
         {
            listCommand.getPlanarRegionCommand(regionIdx).getPlanarRegion(planarRegions.add());
         }

         assertEquals(planarRegions.size(), planarRegionsList.getNumberOfPlanarRegions());
         for (int regionIdx = 0; regionIdx < planarRegions.size(); regionIdx++)
         {
            // First test a few things about the command
            PlanarRegionCommand command = listCommand.getPlanarRegionCommand(regionIdx);
            PlanarRegion expectedRegion = planarRegionsList.getPlanarRegion(regionIdx);
            assertEquals(expectedRegion.getNumberOfConvexPolygons(), command.getConvexPolygons().size());

            for (int polygonIdx = 0; polygonIdx < command.getConvexPolygons().size(); polygonIdx++)
               EuclidCoreTestTools.assertGeometricallyEquals(expectedRegion.getConvexPolygon(polygonIdx), command.getConvexPolygons().get(polygonIdx), 1e-5);

            PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegion, planarRegions.get(regionIdx), 1e-5);
         }
      }
   }

   public static Predicate<DataSet> buildFilter(Predicate<PlannerInput> testSpecificFilter)
   {
      return dataSet -> dataSet.hasPlannerInput() && testSpecificFilter.test(dataSet.getPlannerInput());
   }

   public static Predicate<PlannerInput> getTestableFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsTestable() && plannerInput.containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
   }

   public static String getTestNamePrefix()
   {
      return "a_star";
   }


}
