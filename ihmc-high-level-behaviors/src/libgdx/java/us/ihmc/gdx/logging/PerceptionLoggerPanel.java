package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.perception.HDF5Tools;

import java.util.ArrayList;

public class PerceptionLoggerPanel extends ImGuiPanel
{
   private ImInt topicIndex = new ImInt(0);
   private PerceptionDataLogger logger;
   private PerceptionDataLoader loader;
   private ArrayList<String> topicNames = new ArrayList<>();
   private ArrayList<Integer> topicObjectCounts = new ArrayList<>();

   static final String PERCEPTION_LOG_FILE = "/home/quantum/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.h5";

   private String currentTopic;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLoggerPanel(String panelName)
   {
      super(panelName);
      loader = new PerceptionDataLoader(PERCEPTION_LOG_FILE);
      topicNames = HDF5Tools.getTopicNames(loader.getH5File());
      for (String topic : topicNames)
      {
         topicObjectCounts.add(HDF5Tools.getCount(loader.getH5File(), topic));
      }
      setRenderMethod(this::renderImguiWidgets);
   }

   public void renderImguiWidgets()
   {
      ImGui.text("Perception Log: " + loader.getFilePath());
      String[] names = topicNames.toArray(new String[0]);
      ImGui.combo("Topic Names", topicIndex, names);
      currentTopic = topicNames.get(topicIndex.get());
      ImGui.text("Total Files: " + topicObjectCounts.get(topicIndex.get()));
   }
}
