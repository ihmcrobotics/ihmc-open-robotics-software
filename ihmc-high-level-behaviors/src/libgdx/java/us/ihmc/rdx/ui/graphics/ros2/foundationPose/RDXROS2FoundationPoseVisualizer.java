package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.Box3DMessage;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class RDXROS2FoundationPoseVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ROS2Node ros2Node;

   private final List<ROS2Topic<?>> resultTopics;

   private final Map<IsaacROSFoundationPoseObject, RDXROS2FoundationPoseSettings> settingsMap;

   private final Map<IsaacROSFoundationPoseObject, RDXFoundationPoseResultVisualizer> resultVisualizers;

   public RDXROS2FoundationPoseVisualizer(String title, ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockOffsetEstimator)
   {
      super(title);

      this.ros2Node = ros2Node;

      resultTopics = new ArrayList<>(IsaacROSFoundationPoseObject.values().length);
      settingsMap = new EnumMap<>(IsaacROSFoundationPoseObject.class);
      resultVisualizers = new EnumMap<>(IsaacROSFoundationPoseObject.class);

      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         resultTopics.add(object.topics.ihmcResult());
         settingsMap.put(object, new RDXROS2FoundationPoseSettings(ros2Node, peerClockOffsetEstimator, object));
      }

      setSceneLevels(RDXSceneLevel.VIRTUAL);
   }

   @Override
   public void create()
   {
      super.create();

      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         resultVisualizers.put(object, new RDXFoundationPoseResultVisualizer(ros2Node, object));
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

      for (RDXROS2FoundationPoseSettings settings : settingsMap.values())
         settings.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      for (RDXROS2FoundationPoseSettings settings : settingsMap.values())
         settings.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXFoundationPoseResultVisualizer resultVisualizer : resultVisualizers.values())
      {
         if (sceneLevelCheck(sceneLevels))
            resultVisualizer.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (RDXROS2FoundationPoseSettings settings : settingsMap.values())
         settings.destroy();
   }

   private static class RDXFoundationPoseResultVisualizer implements RenderableProvider
   {
      private final ROS2Subscription<Box3DMessage> resultSubscription;
      private final Box3D latestResult;
      private final RDXBoxVisualizer boxVisualizer;
      private final RDXReferenceFrameGraphic referenceFrameGraphic;

      public RDXFoundationPoseResultVisualizer(ROS2Node ros2Node, IsaacROSFoundationPoseObject object)
      {
         latestResult = new Box3D();
         latestResult.setToNaN();

         boxVisualizer = new RDXBoxVisualizer();
         boxVisualizer.setColor(Color.RED);
         boxVisualizer.setLineWidth(0.01);

         referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);

         resultSubscription = ros2Node.createSubscription2(object.topics.ihmcResult(), message ->
         {
            latestResult.getPose().set(message.getPose());
            latestResult.getSize().set(message.getSize());
            boxVisualizer.generateMesh(latestResult);
            referenceFrameGraphic.getFramePose3D().set(message.getPose());
            referenceFrameGraphic.updateFromFramePose();
         });
      }

      @Override
      public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
      {
         boxVisualizer.update();
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
