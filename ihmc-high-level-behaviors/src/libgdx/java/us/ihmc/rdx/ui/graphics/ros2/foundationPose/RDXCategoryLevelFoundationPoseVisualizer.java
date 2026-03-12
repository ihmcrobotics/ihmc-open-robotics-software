package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.Box3DMessage;
import imgui.ImGui;
import imgui.ImGuiStyle;
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
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class RDXCategoryLevelFoundationPoseVisualizer extends RDXROS2MultiTopicVisualizer
{
   private static final int TABLE_COLUMN_COUNT = 5;

   private final ROS2Node ros2Node;
   private final List<ROS2Topic<?>> resultTopics;

   private final Map<CategoryLevelFoundationPoseObject, RDXCategoryLevelFoundationPoseSettings> settingsMap;
   private final Map<CategoryLevelFoundationPoseObject, CategoryLevelFoundationPoseResultVisualizer> resultVisualizers;
   private final Map<String, List<CategoryLevelFoundationPoseObject>> objectsByCategory;
   private final Set<String> selectedCategories;

   public RDXCategoryLevelFoundationPoseVisualizer(String title,
                                                   ROS2Node ros2Node,
                                                   ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      this(title, ros2Node, ros2ClockOffsetEstimator, Collections.emptySet());
   }

   public RDXCategoryLevelFoundationPoseVisualizer(String title,
                                                   ROS2Node ros2Node,
                                                   ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                                   Set<String> selectedCategories)
   {
      super(title);

      this.ros2Node = ros2Node;

      if (selectedCategories == null)
         this.selectedCategories = Collections.emptySet();
      else
      {
         this.selectedCategories = new LinkedHashSet<>();
         for (String category : selectedCategories)
         {
            if (category != null && !category.isBlank())
               this.selectedCategories.add(category.trim().toLowerCase(Locale.ROOT));
         }
      }

      resultTopics = new ArrayList<>();
      settingsMap = new LinkedHashMap<>();
      resultVisualizers = new LinkedHashMap<>();
      objectsByCategory = new LinkedHashMap<>();

      for (CategoryLevelFoundationPoseObject object : CategoryLevelFoundationPoseObject.VALUES)
      {
         if (!shouldIncludeCategory(object.category))
            continue;

         resultTopics.add(object.topics.ihmcResult());
         settingsMap.put(object, new RDXCategoryLevelFoundationPoseSettings(ros2Node, ros2ClockOffsetEstimator, object));
         objectsByCategory.computeIfAbsent(object.category, key -> new ArrayList<>()).add(object);
      }

      setSceneLevels(RDXSceneLevel.VIRTUAL);
   }

   private boolean shouldIncludeCategory(String category)
   {
      if (selectedCategories.isEmpty())
         return true;

      return selectedCategories.contains(category.toLowerCase(Locale.ROOT));
   }

   @Override
   public void create()
   {
      super.create();

      for (CategoryLevelFoundationPoseObject object : settingsMap.keySet())
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

      for (RDXCategoryLevelFoundationPoseSettings settings : settingsMap.values())
         settings.update();

      for (CategoryLevelFoundationPoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      final ImGuiStyle style = new ImGuiStyle();
      int tableFlags = ImGuiTableFlags.BordersV
                       | ImGuiTableFlags.BordersOuterH
                       | ImGuiTableFlags.NoKeepColumnsVisible
                       | ImGuiTableFlags.RowBg;

      float resetButtonWidth = ImGui.calcTextSize("Reset").x + (2.0f * style.getItemInnerSpacingX()) + 1.0f;
      float distanceWidgetWidth = Math.max(110.0f, resetButtonWidth);

      for (Map.Entry<String, List<CategoryLevelFoundationPoseObject>> entry : objectsByCategory.entrySet())
      {
         String category = entry.getKey();
         List<CategoryLevelFoundationPoseObject> categoryObjects = entry.getValue();

         if (ImGui.treeNode(labels.get(category)))
         {
            String tableName = "CategoryLevelFoundationPoseTable_" + category;

            if (ImGui.beginTable(labels.getHidden(tableName), TABLE_COLUMN_COUNT, tableFlags))
            {
               ImGui.tableSetupColumn(labels.get("Enable"), ImGuiTableColumnFlags.WidthFixed | ImGuiTableColumnFlags.NoHeaderWidth);
               ImGui.tableSetupColumn(labels.get("Instance"), ImGuiTableColumnFlags.WidthStretch);
               ImGui.tableSetupColumn(labels.get("Reset"), ImGuiTableColumnFlags.WidthFixed, resetButtonWidth);
               ImGui.tableSetupColumn(labels.get("Enable Auto Reset"), ImGuiTableColumnFlags.WidthFixed | ImGuiTableColumnFlags.NoHeaderWidth);
               ImGui.tableSetupColumn(labels.get("Auto Reset Distance"), ImGuiTableColumnFlags.WidthFixed, distanceWidgetWidth);

               ImGui.tableHeadersRow();
               ImGui.setItemAllowOverlap();

               for (CategoryLevelFoundationPoseObject object : categoryObjects)
               {
                  RDXCategoryLevelFoundationPoseSettings settings = settingsMap.get(object);
                  if (settings != null)
                     settings.renderAsTableRow();
               }

               ImGui.endTable();
            }

            ImGui.treePop();
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevelCheck(sceneLevels))
         return;

      for (CategoryLevelFoundationPoseObject object : settingsMap.keySet())
      {
         RDXCategoryLevelFoundationPoseSettings settings = settingsMap.get(object);
         if (settings != null && settings.getParameters().getEnabled().getValue())
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
      for (RDXCategoryLevelFoundationPoseSettings settings : settingsMap.values())
         settings.destroy();

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