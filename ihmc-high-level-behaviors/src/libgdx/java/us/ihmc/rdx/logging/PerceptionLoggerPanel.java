package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.FloatBuffer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

public class PerceptionLoggerPanel extends ImGuiPanel
{
   private PerceptionDataLogger logger;
   private PerceptionDataLoader loader;
   private RDXPointCloudRenderer pointCloudRenderer;

   private ArrayList<String> topicNames = new ArrayList<>();
   private ArrayList<Integer> topicObjectCounts = new ArrayList<>();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString(256);
   private final ImInt topicIndex = new ImInt(0);
   private final ImInt objectIndex = new ImInt(0);

   private String currentTopic;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLoggerPanel()
   {
      this("Perception Logging and Loading");
   }

   public PerceptionLoggerPanel(String panelName)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void loadLog(String perceptionLogFile)
   {
      perceptionLogPath.set(perceptionLogFile);
      if (Files.exists(Paths.get(perceptionLogFile)))
      {
         loader = new PerceptionDataLoader(perceptionLogFile);
         topicNames = HDF5Tools.getTopicNames(loader.getHDF5Manager().getFile());
         for (String topic : topicNames)
         {
            topicObjectCounts.add((int) loader.getHDF5Manager().getCount(topic));
         }
      }
      else
      {
         LogTools.warn("Log file does not exist: {}", perceptionLogFile);
      }
   }

   public void setPointCloudRenderer(RDXPointCloudRenderer pointCloudRenderer)
   {
      this.pointCloudRenderer = pointCloudRenderer;
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Perception Log"), perceptionLogPath);
      if (ImGui.button(labels.get("Load log")))
      {
         loadLog(perceptionLogPath.get());
      }

      if (loader != null)
      {
         ImGui.text("Loaded Log: " + loader.getFilePath());
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

   public PerceptionDataLogger getLogger()
   {
      return logger;
   }
}
