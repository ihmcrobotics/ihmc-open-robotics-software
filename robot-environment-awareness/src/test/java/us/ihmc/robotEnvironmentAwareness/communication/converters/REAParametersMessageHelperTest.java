package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import perception_msgs.msg.dds.NormalEstimationParametersMessage;
import perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage;
import perception_msgs.msg.dds.PolygonizerParametersMessage;

public class REAParametersMessageHelperTest
{
   private final Random random = new Random(3289328);
   private final int iterations = 1000;
   
   @Test
   public void testNormalEstimationParameters()
   {
      NormalEstimationParametersMessage input = new NormalEstimationParametersMessage();
      for (int i = 0; i < iterations; i++)
      {
         setToRandom(input, random);
         NormalEstimationParametersMessage output = REAParametersMessageHelper.convertToMessage(REAParametersMessageHelper.convertFromMessage(input));
         Assertions.assertTrue(input.equals(output));
      }
   }  
   
   @Test
   public void testPlanarRegionSegmentationParameters()
   {
      PlanarRegionSegmentationParametersMessage input = new PlanarRegionSegmentationParametersMessage();
      for (int i = 0; i < iterations; i++)
      {
         setToRandom(input, random);
         PlanarRegionSegmentationParametersMessage output = REAParametersMessageHelper.convertToMessage(REAParametersMessageHelper.convertFromMessage(input));
         Assertions.assertTrue(input.equals(output));
      }
   }  
   
   @Test
   public void testPolygonizerParameters()
   {
      PolygonizerParametersMessage input = new PolygonizerParametersMessage();
      for (int i = 0; i < iterations; i++)
      {
         setToRandom(input, random);
         PolygonizerParametersMessage output = REAParametersMessageHelper.convertToMessage(REAParametersMessageHelper.convertFromMessage(input));
         Assertions.assertTrue(input.equals(output));
      }
   }  
   
   private static void setToRandom(NormalEstimationParametersMessage parameters, Random random)
   {      
      parameters.setSearchRadius(random.nextDouble());
      parameters.setMaxDistanceFromPlane(random.nextDouble());
      parameters.setMinConsensusRatio(random.nextDouble());
      parameters.setMaxAverageDeviationRatio(random.nextDouble());
      parameters.setNumberOfIterations(random.nextInt());
      parameters.setEnableLeastSquaresEstimation(random.nextBoolean());
      parameters.setWeightByNumberOfHits(random.nextBoolean());
   }

   private static void setToRandom(PlanarRegionSegmentationParametersMessage parameters, Random random)
   {
      parameters.setSearchRadius(random.nextDouble());
      parameters.setMaxDistanceFromPlane(random.nextDouble());
      parameters.setMaxAngleFromPlane(random.nextDouble());
      parameters.setMinNormalQuality(random.nextDouble());
      parameters.setMinRegionSize(random.nextInt());
      parameters.setMaxStandardDeviation(random.nextDouble());
      parameters.setMinVolumicDensity(random.nextDouble());
   }

   private static void setToRandom(PolygonizerParametersMessage parameters, Random random)
   {
      parameters.setConcaveHullThreshold(random.nextDouble());
      parameters.setMinNumberOfNodes(random.nextInt());
      parameters.setShallowAngleThreshold(random.nextDouble());
      parameters.setPeakAngleThreshold(random.nextDouble());
      parameters.setLengthThreshold(random.nextDouble());
      parameters.setDepthThreshold(random.nextDouble());
      parameters.setCutNarrowPassage(random.nextBoolean());
   }
}
