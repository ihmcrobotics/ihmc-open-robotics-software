package us.ihmc.rdx.ui.graphics.ros2.supervisePose;

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
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.detections.supervisePose.SupervisePoseAPI;
import us.ihmc.perception.detections.supervisePose.SupervisePoseCommunicator.State;
import us.ihmc.perception.detections.supervisePose.SupervisePoseObject;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
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

public class RDXSupervisePoseVisualizer extends RDXROS2MultiTopicVisualizer
{
   private static final int TABLE_COLUMN_COUNT = 5;

   private final ROS2Node ros2Node;
   private final List<ROS2Topic<?>> resultTopics;

   private final Map<SupervisePoseObject, RDXSupervisePoseSettings> settingsMap;
   private final Map<SupervisePoseObject, SupervisePoseResultVisualizer> resultVisualizers;
   private final Map<String, List<SupervisePoseObject>> objectsByCategory;
   private final Set<String> selectedCategories;

   public RDXSupervisePoseVisualizer(String title,
                                     ROS2Node ros2Node,
                                     ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      this(title, ros2Node, ros2ClockOffsetEstimator, Collections.emptySet());
   }

   public RDXSupervisePoseVisualizer(String title,
                                     ROS2Node ros2Node,
                                     ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                     Set<String> selectedCategories)
   {
      super(title);

      this.ros2Node = ros2Node;

      if (selectedCategories == null)
      {
         this.selectedCategories = Collections.emptySet();
      }
      else
      {
         this.selectedCategories = new LinkedHashSet<>();

         for (String category : selectedCategories)
         {
            if (category != null && !category.isBlank())
            {
               this.selectedCategories.add(category.trim()
                                                   .toLowerCase(Locale.ROOT));
            }
         }
      }

      resultTopics = new ArrayList<>();
      settingsMap = new LinkedHashMap<>();
      resultVisualizers = new LinkedHashMap<>();
      objectsByCategory = new LinkedHashMap<>();

      for (SupervisePoseObject object : SupervisePoseObject.VALUES)
      {
         if (!shouldIncludeCategory(object.category))
            continue;

         resultTopics.add(object.topics.ihmcResult());

         settingsMap.put(object,
                         new RDXSupervisePoseSettings(ros2Node,
                                                      ros2ClockOffsetEstimator,
                                                      object));

         objectsByCategory.computeIfAbsent(object.category,
                                           key -> new ArrayList<>())
                          .add(object);
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

      for (SupervisePoseObject object : settingsMap.keySet())
      {
         resultVisualizers.put(object,
                               new SupervisePoseResultVisualizer(ros2Node,
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

      for (RDXSupervisePoseSettings settings : settingsMap.values())
         settings.update();

      for (SupervisePoseResultVisualizer visualizer : resultVisualizers.values())
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

      float resetButtonWidth =
            ImGui.calcTextSize("Reset").x
            + (2.0f * style.getItemInnerSpacingX())
            + 1.0f;

      float distanceWidgetWidth =
            Math.max(110.0f, resetButtonWidth);

      for (Map.Entry<String, List<SupervisePoseObject>> entry : objectsByCategory.entrySet())
      {
         String category = entry.getKey();
         List<SupervisePoseObject> categoryObjects = entry.getValue();

         if (ImGui.treeNode(labels.get(category)))
         {
            String tableName = "SupervisePoseTable_" + category;

            if (ImGui.beginTable(labels.getHidden(tableName),
                                 TABLE_COLUMN_COUNT,
                                 tableFlags))
            {
               ImGui.tableSetupColumn(labels.get("Enable"),
                                      ImGuiTableColumnFlags.WidthFixed
                                      | ImGuiTableColumnFlags.NoHeaderWidth);

               ImGui.tableSetupColumn(labels.get("Instance"),
                                      ImGuiTableColumnFlags.WidthStretch);

               ImGui.tableSetupColumn(labels.get("Reset"),
                                      ImGuiTableColumnFlags.WidthFixed,
                                      resetButtonWidth);

               ImGui.tableSetupColumn(labels.get("Enable Auto Reset"),
                                      ImGuiTableColumnFlags.WidthFixed
                                      | ImGuiTableColumnFlags.NoHeaderWidth);

               ImGui.tableSetupColumn(labels.get("Auto Reset Distance"),
                                      ImGuiTableColumnFlags.WidthFixed,
                                      distanceWidgetWidth);

               ImGui.tableHeadersRow();
               ImGui.setItemAllowOverlap();

               for (SupervisePoseObject object : categoryObjects)
               {
                  RDXSupervisePoseSettings settings = settingsMap.get(object);

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
   public void getRenderables(Array<Renderable> renderables,
                              Pool<Renderable> pool,
                              Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevelCheck(sceneLevels))
         return;

      for (SupervisePoseObject object : settingsMap.keySet())
      {
         RDXSupervisePoseSettings settings = settingsMap.get(object);

         if (settings != null
             && settings.getParameters().getEnabled().getValue())
         {
            SupervisePoseResultVisualizer visualizer =
                  resultVisualizers.get(object);

            if (visualizer != null)
               visualizer.getRenderables(renderables, pool);
         }
      }
   }

   @Override
   public void destroy()
   {
      for (RDXSupervisePoseSettings settings : settingsMap.values())
         settings.destroy();

      for (SupervisePoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.dispose();

      super.destroy();
   }

   /**
    * Displays the combined and per-object overlay images as one hierarchical
    * entry in the perception visualizers panel:
    *
    * SupervisePose Overlay Image
    * ├── Combined
    * └── Category
    *     ├── Instance 1
    *     └── Instance 2
    *
    * Each image visualizer still owns an independent, dockable RDX panel.
    */
   public static class OverlayImageVisualizer extends RDXROS2MultiTopicVisualizer
   {
      private final RDXROS2ImageMessageVisualizer combinedVisualizer;
      private final ImBoolean combinedEnabled = new ImBoolean(false);
      private final RDXPanel panel;

      private final Map<String, List<ObjectOverlayVisualizer>> visualizersByCategory =
            new LinkedHashMap<>();

      private final List<RDXROS2ImageMessageVisualizer> individualVisualizers =
            new ArrayList<>();

      private final List<ROS2Topic<?>> overlayTopics =
            new ArrayList<>();

      private boolean childrenCreated = false;
      private boolean childPanelsRegistered = false;

      public OverlayImageVisualizer(String title,
                                    ROS2Node ros2Node,
                                    Set<String> selectedCategories)
      {
         super(title);

         panel = new RDXPanel(title, this::renderImGuiWidgets);
         panel.getIsShowing().set(false);

         Set<String> normalizedSelectedCategories = normalizeSelectedCategories(selectedCategories);

         combinedVisualizer =
               new RDXROS2ImageMessageVisualizer("SupervisePose Overlay Combined",
                                                 ros2Node,
                                                 SupervisePoseAPI.SUPERVISE_POSE_OVERLAY_IMAGE);

         combinedVisualizer.setActive(false);
         combinedVisualizer.getPanel().getIsShowing().set(false);

         overlayTopics.add(SupervisePoseAPI.SUPERVISE_POSE_OVERLAY_IMAGE);

         for (SupervisePoseObject object : SupervisePoseObject.VALUES)
         {
            if (!shouldIncludeCategory(object.category,
                                       normalizedSelectedCategories))
            {
               continue;
            }

            RDXROS2ImageMessageVisualizer imageVisualizer =
                  new RDXROS2ImageMessageVisualizer("SupervisePose Overlay " + object.titleCaseName,
                                                    ros2Node,
                                                    object.topics.overlayedImage());

            imageVisualizer.setActive(false);
            imageVisualizer.getPanel().getIsShowing().set(false);

            individualVisualizers.add(imageVisualizer);
            overlayTopics.add(object.topics.overlayedImage());

            ObjectOverlayVisualizer objectOverlayVisualizer =
                  new ObjectOverlayVisualizer(object,
                                              imageVisualizer,
                                              new ImBoolean(false));

            visualizersByCategory.computeIfAbsent(object.category,
                                                  ignored -> new ArrayList<>())
                                 .add(objectOverlayVisualizer);
         }
      }

      @Override
      public void create()
      {
         super.create();

         if (!childrenCreated)
         {
            combinedVisualizer.create();

            for (RDXROS2ImageMessageVisualizer visualizer : individualVisualizers)
               visualizer.create();

            childrenCreated = true;
         }

         registerImagePanels();
      }

      /**
       * Registers the combined and per-object image panels with the global panel
       * manager so that each image can be shown and docked independently.
       */
      private void registerImagePanels()
      {
         if (childPanelsRegistered)
            return;

         RDXBaseUI baseUI = RDXBaseUI.getInstance();

         if (baseUI == null)
            return;

         baseUI.getImGuiPanelManager()
               .addPanel(combinedVisualizer.getPanel());

         for (RDXROS2ImageMessageVisualizer visualizer : individualVisualizers)
         {
            baseUI.getImGuiPanelManager()
                  .addPanel(visualizer.getPanel());
         }

         childPanelsRegistered = true;
      }

      @Override
      public void update()
      {
         super.update();

         registerImagePanels();

         combinedVisualizer.update();

         for (RDXROS2ImageMessageVisualizer visualizer : individualVisualizers)
            visualizer.update();
      }

      @Override
      public void updateHeartbeat()
      {
         super.updateHeartbeat();

         /*
          * Synchronizes each child image visualizer's active state with its
          * corresponding checkbox and updates its heartbeat.
          */
         combinedVisualizer.setActive(combinedEnabled.get());
         combinedVisualizer.updateHeartbeat();

         for (List<ObjectOverlayVisualizer> categoryVisualizers : visualizersByCategory.values())
         {
            for (ObjectOverlayVisualizer entry : categoryVisualizers)
            {
               entry.visualizer().setActive(entry.enabled().get());
               entry.visualizer().updateHeartbeat();
            }
         }
      }

      @Override
      public void renderImGuiWidgets()
      {
         renderCombinedControls();

         for (Map.Entry<String, List<ObjectOverlayVisualizer>> categoryEntry
               : visualizersByCategory.entrySet())
         {
            String categoryDisplayName =
                  toTitleCase(categoryEntry.getKey());

            if (ImGui.treeNode(labels.get(categoryDisplayName)))
            {
               for (ObjectOverlayVisualizer objectVisualizer : categoryEntry.getValue())
               {
                  renderObjectControls(objectVisualizer);
               }

               ImGui.treePop();
            }
         }
      }

      private void renderCombinedControls()
      {
         if (ImGui.checkbox(labels.get("Combined"), combinedEnabled))
         {
            boolean enabled = combinedEnabled.get();

            combinedVisualizer.setActive(enabled);
            combinedVisualizer.getPanel().getIsShowing().set(enabled);
         }

         if (combinedEnabled.get())
         {
            ImGui.indent();
            combinedVisualizer.renderImGuiWidgets();
            ImGui.unindent();
         }
      }

      private void renderObjectControls(ObjectOverlayVisualizer objectVisualizer)
      {
         if (ImGui.checkbox(labels.get(objectVisualizer.object().titleCaseName),
                            objectVisualizer.enabled()))
         {
            boolean enabled = objectVisualizer.enabled().get();

            objectVisualizer.visualizer().setActive(enabled);

            objectVisualizer.visualizer().getPanel().getIsShowing().set(enabled);
         }

         if (objectVisualizer.enabled().get())
         {
            ImGui.indent();
            objectVisualizer.visualizer().renderImGuiWidgets();
            ImGui.unindent();
         }
      }

      @Override
      public RDXPanel getPanel()
      {
         /*
          * Returns the dedicated parent panel containing the hierarchical overlay
          * controls. The Combined and per-object image panels are registered
          * separately in registerImagePanels().
          */
         return panel;
      }

      @Override
      public List<ROS2Topic<?>> getTopics()
      {
         return overlayTopics;
      }

      @Override
      public void destroy()
      {
         combinedVisualizer.destroy();

         for (RDXROS2ImageMessageVisualizer visualizer : individualVisualizers)
            visualizer.destroy();

         super.destroy();
      }

      private static Set<String> normalizeSelectedCategories(Set<String> selectedCategories)
      {
         if (selectedCategories == null || selectedCategories.isEmpty())
            return Collections.emptySet();

         Set<String> normalizedCategories = new LinkedHashSet<>();

         for (String category : selectedCategories)
         {
            if (category != null && !category.isBlank())
            {
               normalizedCategories.add(category.trim()
                                                .toLowerCase(Locale.ROOT));
            }
         }

         return normalizedCategories;
      }

      private static boolean shouldIncludeCategory(String category,
                                                   Set<String> selectedCategories)
      {
         if (selectedCategories.isEmpty())
            return true;

         return selectedCategories.contains(category.toLowerCase(Locale.ROOT));
      }

      private static String toTitleCase(String name)
      {
         StringBuilder result = new StringBuilder();

         for (String word : name.split("_"))
         {
            if (!result.isEmpty())
               result.append(' ');

            result.append(Character.toUpperCase(word.charAt(0)));

            if (word.length() > 1)
               result.append(word.substring(1));
         }

         return result.toString();
      }

      private record ObjectOverlayVisualizer(SupervisePoseObject object,
                                             RDXROS2ImageMessageVisualizer visualizer,
                                             ImBoolean enabled)
      {
      }
   }

   private static class SupervisePoseResultVisualizer implements RenderableProvider
   {
      private final ROS2Subscription<Box3DMessage> resultSubscription;
      private final ROS2Subscription<std_msgs.msg.dds.Byte> stateSubscription;

      private final Box3D latestResult;
      private State state;

      private final RDXBoxVisualizer boxVisualizer;
      private final RDXReferenceFrameGraphic referenceFrameGraphic;

      public SupervisePoseResultVisualizer(ROS2Node ros2Node,
                                           SupervisePoseObject object,
                                           ImGuiAveragedFrequencyText frequencyText)
      {
         latestResult = new Box3D();
         latestResult.setToNaN();

         state = State.DISABLED;

         boxVisualizer = new RDXBoxVisualizer();
         boxVisualizer.setColor(Color.RED);
         boxVisualizer.setLineWidth(0.01);

         referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);

         stateSubscription =
               ros2Node.createSubscription2(object.topics.ihmcState(),
                                            message -> state = State.fromByte(message.getData()));

         resultSubscription =
               ros2Node.createSubscription2(object.topics.ihmcResult(),
                                            message ->
                                            {
                                               frequencyText.ping();

                                               latestResult.getPose()
                                                           .set(message.getPose());

                                               latestResult.getSize()
                                                           .set(message.getSize());

                                               referenceFrameGraphic.getFramePose3D()
                                                                    .set(message.getPose());
                                            });
      }

      public void update()
      {
         boxVisualizer.setColor(switch (state)
                                {
                                   case DISABLED -> Color.RED;
                                   case ESTIMATING_POSE -> Color.ORANGE;
                                   case TRACKING -> Color.GREEN;
                                });

         boxVisualizer.generateMesh(latestResult);
         boxVisualizer.update();
         referenceFrameGraphic.updateFromFramePose();
      }

      @Override
      public void getRenderables(Array<Renderable> renderables,
                                 Pool<Renderable> pool)
      {
         boxVisualizer.getRenderables(renderables, pool);
         referenceFrameGraphic.getRenderables(renderables, pool);
      }

      public void dispose()
      {
         boxVisualizer.dispose();
         referenceFrameGraphic.dispose();

         resultSubscription.remove();
         stateSubscription.remove();
      }
   }
}