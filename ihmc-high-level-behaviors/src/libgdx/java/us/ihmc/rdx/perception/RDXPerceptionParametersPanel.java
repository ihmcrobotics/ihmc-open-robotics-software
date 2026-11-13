package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.perception.geometry.ConcaveHullFactoryParameters;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.TerrainMapParameters;
import us.ihmc.perception.rapidRegions.PolygonizerParameters;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * Tuners for the perception parameters that a remote process serves over ROS 2. Each parameter set is a child panel.
 * The height map, terrain map, and depth image filtering parameters are served by the ActiveMappingParameterToolBox
 * (GpuMappingThread). The rapid regions, polygonizer, and concave hull parameters are served by the
 * RapidPlanarRegionsExtractionThread.
 */
public class RDXPerceptionParametersPanel extends RDXPanel
{
   /** Low rate because WiFi bandwidth to the robot is limited. */
   private static final double ROS2_SYNC_FREQUENCY = 2.0;

   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private final Throttler ros2Throttler = new Throttler().setFrequency(ROS2_SYNC_FREQUENCY);

   public RDXPerceptionParametersPanel(ROS2Node ros2Node)
   {
      super("Perception Parameters");
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Node);

      addTuner("Height Map Parameters", new HeightMapParameters(), PerceptionComms.HEIGHT_MAP_PARAMETERS);
      addTuner("Terrain Map Parameters", new TerrainMapParameters(), PerceptionComms.TERRAIN_MAP_PARAMETERS);
      addTuner("Depth Image Filtering Parameters", new DepthImageFilteringParameters(), PerceptionComms.DEPTH_IMAGE_FILTERING_PARAMETERS);
      addTuner("Rapid Regions Parameters", new RapidRegionsExtractorParameters(), PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS);
      addTuner("Polygonizer Parameters", new PolygonizerParameters(), PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS);
      addTuner("Concave Hull Parameters", new ConcaveHullFactoryParameters(), PerceptionComms.PERSPECTIVE_CONCAVE_HULL_FACTORY_PARAMETERS);
   }

   private void addTuner(String panelName, StoredPropertySetBasics parameters, StoredPropertySetROS2TopicPair topicPair)
   {
      RDXStoredPropertySetTuner tuner = new RDXStoredPropertySetTuner(panelName);
      tuner.create(parameters, false);
      remotePropertySets.registerRemotePropertySet(parameters, topicPair);
      addChild(tuner);
   }

   /** Call every frame to sync the parameters with the remote process. */
   public void update()
   {
      if (ros2Throttler.run())
         remotePropertySets.setPropertyChanged();
   }
}
