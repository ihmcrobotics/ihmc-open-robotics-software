package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.avatar.sensors.realsense.MapsenseTools;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1PlanarRegionsVisualizer extends ImGuiGDXROS1Visualizer implements RenderableProvider
{
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private DRCRobotModel robotModel = null;
   private final String topic;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("Received", 1000, 230, 20);
   private final ImGuiPlot delayPlot = new ImGuiPlot("Delay", 1000, 230, 20);

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private DelayFixedPlanarRegionsSubscription delayFixedPlanarRegionsSubscription;
   private ResettableExceptionHandlingExecutorService executorService;
   private AbstractRosTopicSubscriber<RawGPUPlanarRegionList> subscriber;

   public GDXROS1PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, String topic)
   {
      super(title);
      this.topic = topic;

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor("ROS1PlanarRegionsSubscriber", daemon, queueSize);

      gpuPlanarRegionUpdater.attachROS2Tuner(ros2Node);
   }

   public GDXROS1PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, DRCRobotModel robotModel, String topic)
   {
      super(title);
      this.robotModel = robotModel;
      this.topic = topic;

      delayFixedPlanarRegionsSubscription = MapsenseTools.subscribeToPlanarRegionsWithDelayCompensation(ros2Node, robotModel, topic, planarRegionsList ->
      {
         ++receivedCount;
         planarRegionsGraphic.generateMeshes(planarRegionsList);
      });
      delayFixedPlanarRegionsSubscription.setEnabled(isActive());

      gpuPlanarRegionUpdater.attachROS2Tuner(ros2Node);
   }

   private void acceptRawGPUPlanarRegionsList(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
            planarRegionsGraphic.generateMeshes(planarRegionsList);
         });
      }
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      if (robotModel != null)
      {
         delayFixedPlanarRegionsSubscription.subscribe(ros1Node);
      }
      else
      {
         subscriber = MapsenseTools.createROS1Callback(topic, ros1Node, this::acceptRawGPUPlanarRegionsList);
      }
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      if (robotModel != null)
      {
         delayFixedPlanarRegionsSubscription.unsubscribe(ros1Node);
      }
      else
      {
         ros1Node.removeSubscriber(subscriber);
      }
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      if (robotModel != null)
      {
         delayFixedPlanarRegionsSubscription.setEnabled(active);
      }
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         planarRegionsGraphic.update();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
      if (delayFixedPlanarRegionsSubscription != null)
      {
         delayFixedPlanarRegionsSubscription.setEnabled(isActive());
         ImGui.text("Delay:");
         delayPlot.render((float) delayFixedPlanarRegionsSubscription.getDelay());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      if (delayFixedPlanarRegionsSubscription != null)
      {
         delayFixedPlanarRegionsSubscription.destroy();
      }
      if (executorService != null)
      {
         executorService.destroy();
      }
      planarRegionsGraphic.destroy();
   }
}
