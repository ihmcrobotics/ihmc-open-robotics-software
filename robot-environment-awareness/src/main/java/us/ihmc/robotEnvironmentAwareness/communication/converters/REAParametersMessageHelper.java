package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage;
import controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage;
import perception_msgs.msg.dds.NormalEstimationParametersMessage;
import perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage;
import perception_msgs.msg.dds.PolygonizerParametersMessage;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

public class REAParametersMessageHelper
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

   public static ConcaveHullFactoryParameters convertFromMessage(ConcaveHullFactoryParametersMessage message)
   {
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();

      parameters.setEdgeLengthThreshold(message.getEdgeLengthThreshold());
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(message.getRemoveAllTrianglesWithTwoBorderEdges());
      parameters.setAllowSplittingConcaveHull(message.getAllowSplittingConcaveHull());
      parameters.setMaxNumberOfIterations(message.getMaxNumberOfIterations());
      parameters.setTriangulationTolerance(message.getTriangulationTolerance());

      return parameters;
   }

   public static NormalEstimationParametersMessage convertToMessage(NormalEstimationParameters parameters)
   {
      NormalEstimationParametersMessage message = new NormalEstimationParametersMessage();
      
      message.setSearchRadius(parameters.getSearchRadius());
      message.setMaxDistanceFromPlane(parameters.getMaxDistanceFromPlane());
      message.setMinConsensusRatio(parameters.getMinConsensusRatio());
      message.setMaxAverageDeviationRatio(parameters.getMaxAverageDeviationRatio());
      message.setNumberOfIterations(parameters.getNumberOfIterations());
      message.setEnableLeastSquaresEstimation(parameters.isLeastSquaresEstimationEnabled());
      message.setWeightByNumberOfHits(parameters.isWeightByNumberOfHits());

      return message;
   }

   public static PlanarRegionSegmentationParametersMessage convertToMessage(PlanarRegionSegmentationParameters parameters)
   {
      PlanarRegionSegmentationParametersMessage message = new PlanarRegionSegmentationParametersMessage();

      message.setSearchRadius(parameters.getSearchRadius());
      message.setMaxDistanceFromPlane(parameters.getMaxDistanceFromPlane());
      message.setMaxAngleFromPlane(parameters.getMaxAngleFromPlane());
      message.setMinNormalQuality(parameters.getMinNormalQuality());
      message.setMinRegionSize(parameters.getMinRegionSize());
      message.setMaxStandardDeviation(parameters.getMaxStandardDeviation());
      message.setMinVolumicDensity(parameters.getMinVolumicDensity());

      return message;
   }

   public static PolygonizerParametersMessage convertToMessage(PolygonizerParameters parameters)
   {
      PolygonizerParametersMessage message = new PolygonizerParametersMessage();

      message.setConcaveHullThreshold(parameters.getConcaveHullThreshold());
      message.setMinNumberOfNodes(parameters.getMinNumberOfNodes());
      message.setShallowAngleThreshold(parameters.getShallowAngleThreshold());
      message.setPeakAngleThreshold(parameters.getPeakAngleThreshold());
      message.setLengthThreshold(parameters.getLengthThreshold());
      message.setDepthThreshold(parameters.getDepthThreshold());
      message.setCutNarrowPassage(parameters.getCutNarrowPassage());

      return message;
   }

   public static ConcaveHullFactoryParametersMessage convertToMessage(ConcaveHullFactoryParameters parameters)
   {
      ConcaveHullFactoryParametersMessage message = new ConcaveHullFactoryParametersMessage();

      message.setEdgeLengthThreshold(parameters.getEdgeLengthThreshold());
      message.setRemoveAllTrianglesWithTwoBorderEdges(parameters.getRemoveAllTrianglesWithTwoBorderEdges());
      message.setAllowSplittingConcaveHull(parameters.getAllowSplittingConcaveHull());
      message.setMaxNumberOfIterations(parameters.getMaxNumberOfIterations());
      message.setTriangulationTolerance(parameters.getTriangulationTolerance());

      return message;
   }

   public static void setToDefaults(NormalEstimationParametersMessage message)
   {
      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();

      message.setSearchRadius(normalEstimationParameters.getSearchRadius());
      message.setMaxDistanceFromPlane(normalEstimationParameters.getMaxDistanceFromPlane());
      message.setMinConsensusRatio(normalEstimationParameters.getMinConsensusRatio());
      message.setMaxAverageDeviationRatio(normalEstimationParameters.getMaxAverageDeviationRatio());
      message.setNumberOfIterations(normalEstimationParameters.getNumberOfIterations());
      message.setEnableLeastSquaresEstimation(normalEstimationParameters.isLeastSquaresEstimationEnabled());
      message.setWeightByNumberOfHits(normalEstimationParameters.isWeightByNumberOfHits());
   }

   public static void setToDefaults(PlanarRegionSegmentationParametersMessage message)
   {
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();

      message.setSearchRadius(parameters.getSearchRadius());
      message.setMaxDistanceFromPlane(parameters.getMaxDistanceFromPlane());
      message.setMaxAngleFromPlane(parameters.getMaxAngleFromPlane());
      message.setMinNormalQuality(parameters.getMinNormalQuality());
      message.setMinRegionSize(parameters.getMinRegionSize());
      message.setMaxStandardDeviation(parameters.getMaxStandardDeviation());
      message.setMinVolumicDensity(parameters.getMinVolumicDensity());
   }

   public static void setToDefaults(PolygonizerParametersMessage message)
   {
      PolygonizerParameters parameters = new PolygonizerParameters();

      message.setConcaveHullThreshold(parameters.getConcaveHullThreshold());
      message.setMinNumberOfNodes(parameters.getMinNumberOfNodes());
      message.setShallowAngleThreshold(parameters.getShallowAngleThreshold());
      message.setPeakAngleThreshold(parameters.getPeakAngleThreshold());
      message.setLengthThreshold(parameters.getLengthThreshold());
      message.setDepthThreshold(parameters.getDepthThreshold());
      message.setCutNarrowPassage(parameters.getCutNarrowPassage());
   }
}
