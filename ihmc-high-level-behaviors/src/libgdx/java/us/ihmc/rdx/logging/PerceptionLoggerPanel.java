package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HDF5Tools;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;

import java.nio.FloatBuffer;
import java.util.ArrayList;

public class PerceptionLoggerPanel extends ImGuiPanel
{

   private PerceptionDataLogger logger;
   private PerceptionDataLoader loader;
   private RDXPointCloudRenderer pointCloudRenderer;

   private ArrayList<String> topicNames = new ArrayList<>();
   private ArrayList<Integer> topicObjectCounts = new ArrayList<>();

   private ImInt topicIndex = new ImInt(0);
   private ImInt objectIndex = new ImInt(0);

   static final String PERCEPTION_LOG_FILE = "/home/bmishra/Workspace/Data/Sensor_Logs/experimental.hdf5";

   private String currentTopic;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLoggerPanel(String panelName)
   {
      super(panelName);

      loader = new PerceptionDataLoader(PERCEPTION_LOG_FILE);
      topicNames = HDF5Tools.getTopicNames(loader.getHDF5Manager().getFile());
      for (String topic : topicNames)
      {
         topicObjectCounts.add((int) loader.getHDF5Manager().getCount(topic));
      }
      setRenderMethod(this::renderImguiWidgets);
   }

   public void setPointCloudRenderer(RDXPointCloudRenderer pclRenderer)
   {
      this.pointCloudRenderer = pclRenderer;
   }

   public void renderImguiWidgets()
   {
      ImGui.text("Perception Log: " + loader.getFilePath());
      String[] names = topicNames.toArray(new String[0]);
      ImGui.combo("Topic Names", topicIndex, names);
      currentTopic = topicNames.get(topicIndex.get());
      ImGui.text("Total Files: " + topicObjectCounts.get(topicIndex.get()));

      ImGui.sliderInt("Object Index", objectIndex.getData(), 0, 10);
      if (ImGui.button("Load Next"))
      {
         FloatBuffer pointsBuffer = loader.loadCompressedPointCloud("/os_cloud_node/points", objectIndex.get());

         LogTools.info("Float Buffer: {} {} {}", pointsBuffer.get(0), pointsBuffer.get(1), pointsBuffer.get(2));

         pointCloudRenderer.getVertexBuffer().put(pointsBuffer);
         pointCloudRenderer.updateMeshFastest();
      }
   }
}
