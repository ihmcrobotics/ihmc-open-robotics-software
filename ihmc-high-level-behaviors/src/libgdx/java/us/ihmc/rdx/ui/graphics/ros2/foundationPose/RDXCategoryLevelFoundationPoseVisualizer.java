package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.Box3DMessage;
import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.detections.foundationPose.CategoryLevelFoundationPoseObject;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class RDXCategoryLevelFoundationPoseVisualizer extends RDXROS2MultiTopicVisualizer
{
   private static final int TABLE_COLUMN_COUNT = 4;

   private final ROS2Node ros2Node;
   private final List<ROS2Topic<?>> resultTopics;

   private final Map<CategoryLevelFoundationPoseObject, Boolean> visibleMap;
   private final Map<CategoryLevelFoundationPoseObject, CategoryLevelFoundationPoseResultVisualizer> resultVisualizers;

   public RDXCategoryLevelFoundationPoseVisualizer(String title,
                                                   ROS2Node ros2Node,
                                                   ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      super(title);

      this.ros2Node = ros2Node;

      resultTopics = new ArrayList<>(CategoryLevelFoundationPoseObject.VALUES.length);
      visibleMap = new LinkedHashMap<>();
      resultVisualizers = new LinkedHashMap<>();

      for (CategoryLevelFoundationPoseObject object : CategoryLevelFoundationPoseObject.VALUES)
      {
         resultTopics.add(object.topics.ihmcResult());
         visibleMap.put(object, true);
      }

      setSceneLevels(RDXSceneLevel.VIRTUAL);
   }

   @Override
   public void create()
   {
      super.create();

      for (CategoryLevelFoundationPoseObject object : CategoryLevelFoundationPoseObject.VALUES)
      {
         resultVisualizers.put(object,
                               new CategoryLevelFoundationPoseResultVisualizer(ros2Node,
                                                                               object,
                                                                               getFrequency(object.topics.ihmcResult())));
      }
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return resultTopics;
   }

   @Override
   public void update()
   {
      super.update();

      for (CategoryLevelFoundationPoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      int tableFlags = ImGuiTableFlags.BordersV | ImGuiTableFlags.BordersOuterH | ImGuiTableFlags.RowBg;

      if (ImGui.beginTable(labels.getHidden("CategoryLevelFoundationPoseTable"), TABLE_COLUMN_COUNT, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Show"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Name"), ImGuiTableColumnFlags.WidthStretch);
         ImGui.tableSetupColumn(labels.get("Category"), ImGuiTableColumnFlags.WidthStretch);
         ImGui.tableSetupColumn(labels.get("Instance"), ImGuiTableColumnFlags.WidthStretch);
         ImGui.tableHeadersRow();

         for (CategoryLevelFoundationPoseObject object : CategoryLevelFoundationPoseObject.VALUES)
         {
            ImGui.tableNextRow();

            ImGui.tableSetColumnIndex(0);
            boolean visible = visibleMap.get(object);
            if (ImGui.checkbox(labels.get("show_" + object.name()), visible))
               visibleMap.put(object, !visible);

            ImGui.tableSetColumnIndex(1);
            ImGui.text(object.titleCaseName);

            ImGui.tableSetColumnIndex(2);
            ImGui.text(object.category);

            ImGui.tableSetColumnIndex(3);
            ImGui.text(object.instance);
         }

         ImGui.endTable();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevelCheck(sceneLevels))
         return;

      for (CategoryLevelFoundationPoseObject object : CategoryLevelFoundationPoseObject.VALUES)
      {
         if (Boolean.TRUE.equals(visibleMap.get(object)))
         {
            CategoryLevelFoundationPoseResultVisualizer visualizer = resultVisualizers.get(object);
            if (visualizer != null)
               visualizer.getRenderables(renderables, pool);
         }
      }
   }

   @Override
   public void destroy()
   {
      for (CategoryLevelFoundationPoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.dispose();
   }

   private static class CategoryLevelFoundationPoseResultVisualizer implements RenderableProvider
   {
      private final ROS2Subscription<Box3DMessage> resultSubscription;
      private final Box3D latestResult;
      private final RDXBoxVisualizer boxVisualizer;
      private final RDXReferenceFrameGraphic referenceFrameGraphic;

      public CategoryLevelFoundationPoseResultVisualizer(ROS2Node ros2Node,
                                                         CategoryLevelFoundationPoseObject object,
                                                         ImGuiAveragedFrequencyText frequencyText)
      {
         latestResult = new Box3D();
         latestResult.setToNaN();

         boxVisualizer = new RDXBoxVisualizer();
         boxVisualizer.setColor(Color.RED);
         boxVisualizer.setLineWidth(0.01);

         referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);

         resultSubscription = ros2Node.createSubscription2(object.topics.ihmcResult(), message ->
         {
            frequencyText.ping();

            latestResult.getPose().set(message.getPose());
            latestResult.getSize().set(message.getSize());
            boxVisualizer.generateMesh(latestResult);
            referenceFrameGraphic.getFramePose3D().set(message.getPose());
         });
      }

      public void update()
      {
         boxVisualizer.update();
         referenceFrameGraphic.updateFromFramePose();
      }

      @Override
      public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
      {
         boxVisualizer.getRenderables(renderables, pool);
         referenceFrameGraphic.getRenderables(renderables, pool);
      }

      public void dispose()
      {
         boxVisualizer.dispose();
         referenceFrameGraphic.dispose();
         resultSubscription.remove();
      }
   }
}