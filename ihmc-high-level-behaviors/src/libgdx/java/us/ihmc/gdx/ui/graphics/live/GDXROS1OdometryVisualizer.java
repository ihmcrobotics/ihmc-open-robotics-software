package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import geometry_msgs.PoseStamped;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayList;

public class GDXROS1OdometryVisualizer extends ImGuiGDXROS1Visualizer implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private AbstractRosTopicSubscriber<PoseStamped> subscriber;
   private final String topic;
   private PoseStamped pose;
   private ArrayList<ModelInstance> poseModels = new ArrayList<>();

   private volatile Runnable toRender = null;

   private ModelInstance modelInstance;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS1OdometryVisualizer(String title, String topic)
   {
      super(title);
      this.topic = topic;
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<PoseStamped>(PoseStamped._TYPE)
      {
         @Override
         public void onNewMessage(PoseStamped pose)
         {

            GDXROS1OdometryVisualizer.this.pose = pose;
            ++receivedCount;

            queueRenderPosesAsync(pose);
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
   }

   private void queueRenderPosesAsync(PoseStamped pose)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(pose));
   }

   public synchronized void generateMeshes(PoseStamped pose)
   {
      toRender = () ->
      {
         modelInstance = GDXModelPrimitives.createCoordinateFrameInstance(0.1);
         modelInstance.transform.translate((float) pose.getPose().getPosition().getX(),
                                           (float) pose.getPose().getPosition().getY(),
                                           (float) pose.getPose().getPosition().getZ());
         poseModels.add(modelInstance);
      };
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
   }

   @Override
   public void renderGraphics()
   {
      super.renderGraphics();
      if (isActive())
      {
         if (toRender != null)
         {
            toRender.run();
            toRender = null;
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance model : poseModels)
      {
         model.getRenderables(renderables, pool);
      }
   }
}
