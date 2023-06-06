package us.ihmc.rdx.perception;

import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;

public class RDXPlanarRegionsExtractorTools
{
   public static RapidPlanarRegionsExtractor create(OpenCLManager openCLManager, RDXHighLevelDepthSensorSimulator depthSensorSimulator)
   {
      RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      rapidPlanarRegionsExtractor.create(openCLManager,
                                         depthSensorSimulator.getLowLevelSimulator().getImageHeight(),
                                         depthSensorSimulator.getLowLevelSimulator().getImageWidth(),
                                         depthSensorSimulator.getLowLevelSimulator().getFocalLengthPixels().get(),
                                         depthSensorSimulator.getLowLevelSimulator().getFocalLengthPixels().get(),
                                         depthSensorSimulator.getLowLevelSimulator().getPrincipalOffsetXPixels().get(),
                                         depthSensorSimulator.getLowLevelSimulator().getPrincipalOffsetYPixels().get());
      return rapidPlanarRegionsExtractor;
   }
}
