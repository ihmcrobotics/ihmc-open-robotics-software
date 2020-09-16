package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.NormalEstimationParametersMessage;
import controller_msgs.msg.dds.PlanarRegionSegmentationParametersMessage;
import controller_msgs.msg.dds.PolygonizerParametersMessage;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

public class REAParametersConverter
{
   public static NormalEstimationParameters convertFromMessage(NormalEstimationParametersMessage message)
   {
      NormalEstimationParameters parameters = new NormalEstimationParameters();

      parameters.setSearchRadius(message.getSearchRadius());
      parameters.setMaxDistanceFromPlane(message.getMaxDistanceFromPlane());
      parameters.setMinConsensusRatio(message.getMinConsensusRatio());
      parameters.setMaxAverageDeviationRatio(message.getMaxAverageDeviationRatio());
      parameters.setNumberOfIterations(message.getNumberOfIterations());
      parameters.enableLeastSquaresEstimation(message.getEnableLeastSquaresEstimation());
      parameters.weightByNumberOfHits(message.getWeightByNumberOfHits());

      return parameters;
   }

   public static PlanarRegionSegmentationParameters convertFromMessage(PlanarRegionSegmentationParametersMessage message)
   {
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();

      parameters.setSearchRadius(message.getSearchRadius());
      parameters.setMaxDistanceFromPlane(message.getMaxDistanceFromPlane());
      parameters.setMaxAngleFromPlane(message.getMaxAngleFromPlane());
      parameters.setMinNormalQuality(message.getMinNormalQuality());
      parameters.setMinRegionSize(message.getMinRegionSize());
      parameters.setMaxStandardDeviation(message.getMaxStandardDeviation());
      parameters.setMinVolumicDensity(message.getMinVolumicDensity());

      return parameters;
   }

   public static PolygonizerParameters convertFromMessage(PolygonizerParametersMessage message)
   {
      PolygonizerParameters parameters = new PolygonizerParameters();

      parameters.setConcaveHullThreshold(message.getConcaveHullThreshold());
      parameters.setMinNumberOfNodes(message.getMinNumberOfNodes());
      parameters.setShallowAngleThreshold(message.getShallowAngleThreshold());
      parameters.setPeakAngleThreshold(message.getPeakAngleThreshold());
      parameters.setLengthThreshold(message.getLengthThreshold());
      parameters.setDepthThreshold(message.getDepthThreshold());
      parameters.setCutNarrowPassage(message.getCutNarrowPassage());

      return parameters;
   }
}
