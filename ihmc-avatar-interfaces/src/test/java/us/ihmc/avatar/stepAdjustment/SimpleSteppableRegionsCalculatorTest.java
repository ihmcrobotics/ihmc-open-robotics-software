package us.ihmc.avatar.stepAdjustment;

import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.footstepPlanning.PlanarRegionEndToEndConversionTest;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class SimpleSteppableRegionsCalculatorTest
{
   private static final double minRegionArea = 0.005;
   private static final double maxRegionAngle = Math.toRadians(45.0);

   @Test
   public void testFromDataSets()
   {
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(PlanarRegionEndToEndConversionTest.buildFilter(PlanarRegionEndToEndConversionTest.getTestableFilter()));

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      PlanarRegionsListCommand listCommand = new PlanarRegionsListCommand();
      SimpleSteppableRegionsCalculator calculatorToTest = new SimpleSteppableRegionsCalculator(() -> minRegionArea, () -> maxRegionAngle);

      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
         PlanarRegionsListMessage listMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         listCommand.setFromMessage(listMessage);

         SimpleSteppableRegionsCalculator calculatorToTestAgainst = new SimpleSteppableRegionsCalculator(() -> minRegionArea, () -> maxRegionAngle);
         calculatorToTestAgainst.consume(listCommand);
         calculatorToTest.consume(listCommand);

         List<PlanarRegion> filteredRegions = new ArrayList<>();
         for (int regionIdx = 0; regionIdx < planarRegionsList.getNumberOfPlanarRegions(); regionIdx++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIdx);
            if (PlanarRegionTools.computePlanarRegionArea(planarRegion) < minRegionArea)
               continue;

            if (Math.abs(planarRegion.getNormalZ()) < Math.cos(maxRegionAngle))
               continue;

            filteredRegions.add(planarRegion);
         }

         assertEquals(filteredRegions.size(), calculatorToTest.getSteppableRegions().size());
         assertEquals(filteredRegions.size(), calculatorToTestAgainst.getSteppableRegions().size());
         for (int regionIdx = 0; regionIdx < filteredRegions.size(); regionIdx++)
         {
            // First test a few things about the command
            PlanarRegion expectedRegion = filteredRegions.get(regionIdx);

            PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegion, calculatorToTest.getSteppableRegions().get(regionIdx), 1e-5);
            PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegion, calculatorToTestAgainst.getSteppableRegions().get(regionIdx), 1e-5);
         }
      }
   }
}
