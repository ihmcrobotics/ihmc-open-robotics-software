package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.Box3DMessage;
import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicator.State;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class RDXIsaacROSFoundationPoseVisualizer extends RDXROS2MultiTopicVisualizer
{
   private static final int TABLE_COLUMN_COUNT = 5;

   private final ROS2Node ros2Node;
   private final List<ROS2Topic<?>> resultTopics;

   private final Map<IsaacROSFoundationPoseObject, RDXIsaacROSFoundationPoseSettings> settingsMap;
   private final Map<IsaacROSFoundationPoseObject, RDXIsaacROSFoundationPoseResultVisualizer> resultVisualizers;

   public RDXIsaacROSFoundationPoseVisualizer(String title, ROS2Node ros2Node, ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator)
   {
      super(title);

      this.ros2Node = ros2Node;

      resultTopics = new ArrayList<>(IsaacROSFoundationPoseObject.values().length);
      settingsMap = new EnumMap<>(IsaacROSFoundationPoseObject.class);
      resultVisualizers = new EnumMap<>(IsaacROSFoundationPoseObject.class);

      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         resultTopics.add(object.topics.ihmcResult());
         settingsMap.put(object, new RDXIsaacROSFoundationPoseSettings(ros2Node, ros2ClockOffsetEstimator, object));
      }

      setSceneLevels(RDXSceneLevel.VIRTUAL);
   }

   @Override
   public void create()
   {
      super.create();
      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         resultVisualizers.put(object, new RDXIsaacROSFoundationPoseResultVisualizer(ros2Node, object, getFrequency(object.topics.ihmcResult())));
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

      for (RDXIsaacROSFoundationPoseSettings settings : settingsMap.values())
         settings.update();

      for (RDXIsaacROSFoundationPoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      final ImGuiStyle style = new ImGuiStyle();
      int tableFlags = ImGuiTableFlags.BordersV | ImGuiTableFlags.BordersOuterH | ImGuiTableFlags.NoKeepColumnsVisible | ImGuiTableFlags.RowBg;

      if (ImGui.beginTable(labels.getHidden("Settings Table"), TABLE_COLUMN_COUNT, tableFlags))
      {
         float widgetWidth = ImGui.calcTextSize("Reset").x + (2.0f * style.getItemInnerSpacingX()) + 1.0f;
         ImGui.tableSetupColumn(labels.get("Enable"), ImGuiTableColumnFlags.WidthFixed | ImGuiTableColumnFlags.NoHeaderWidth);
         ImGui.tableSetupColumn(labels.get("Object"), ImGuiTableColumnFlags.WidthStretch);
         ImGui.tableSetupColumn(labels.get("Reset"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);
         ImGui.tableSetupColumn(labels.get("Enable Auto Reset"), ImGuiTableColumnFlags.WidthFixed | ImGuiTableColumnFlags.NoHeaderWidth);
         ImGui.tableSetupColumn(labels.get("Auto Reset Distance"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);

         // Render header
         ImGui.tableHeadersRow();
         ImGui.setItemAllowOverlap();

         // Render settings
         for (RDXIsaacROSFoundationPoseSettings settings : settingsMap.values())
            settings.renderAsTableRow();

         ImGui.endTable();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         if (settingsMap.get(object).getParameters().getEnabled().getValue() && sceneLevelCheck(sceneLevels))
         {
            resultVisualizers.get(object).getRenderables(renderables, pool);
         }
      }
   }

   @Override
   public void destroy()
   {
      for (RDXIsaacROSFoundationPoseSettings settings : settingsMap.values())
         settings.destroy();

      for (RDXIsaacROSFoundationPoseResultVisualizer visualizer : resultVisualizers.values())
         visualizer.dispose();
   }

   private static class RDXIsaacROSFoundationPoseResultVisualizer implements RenderableProvider
   {
      private final ROS2Node ros2Node;
      private final ROS2Subscription<Box3DMessage> resultSubscription;
      private final Box3D latestResult;

      private final ROS2Subscription<std_msgs.Byte> stateSubscription;
      private State state;

      private final RDXBoxVisualizer boxVisualizer;
      private final RDXReferenceFrameGraphic referenceFrameGraphic;

      public RDXIsaacROSFoundationPoseResultVisualizer(ROS2Node ros2Node, IsaacROSFoundationPoseObject object, ImGuiAveragedFrequencyText frequencyText)
      {
         this.ros2Node = ros2Node;
         latestResult = new Box3D();
         latestResult.setToNaN();

         state = State.DISABLED;

         boxVisualizer = new RDXBoxVisualizer();
         boxVisualizer.setColor(Color.RED);
         boxVisualizer.setLineWidth(0.01);

         referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);

         stateSubscription = ros2Node.createSubscription(object.topics.ihmcState(), reader -> state = State.fromByte(reader.read().getData()));

         resultSubscription = ros2Node.createSubscription(object.topics.ihmcResult(), reader ->
         {
            frequencyText.ping();

            var message = reader.read();
            latestResult.getPose().set(message.getPose().getPose());
            latestResult.getSize().set(message.getSize().getVector());
            referenceFrameGraphic.getFramePose3D().set(message.getPose().getPose());
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
      public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
      {
         boxVisualizer.getRenderables(renderables, pool);
         referenceFrameGraphic.getRenderables(renderables, pool);
      }

      public void dispose()
      {
         boxVisualizer.dispose();
         referenceFrameGraphic.dispose();
         ros2Node.destroySubscription(resultSubscription);
         ros2Node.destroySubscription(stateSubscription);
      }
   }
}
