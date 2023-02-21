package us.ihmc.rdx.ui.graphics.ros1;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import geometry_msgs.PoseStamped;
import imgui.internal.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXROS1Visualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayList;

@Deprecated
public class RDXROS1OdometryVisualizer extends RDXROS1Visualizer implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private AbstractRosTopicSubscriber<PoseStamped> subscriber;
   private final String topic;
   private ReferenceFrame frame;
   private PoseStamped pose;
   private final FramePose3D framePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();

   private volatile Runnable toRender = null;

   private ModelInstance modelInstance;

   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   public RDXROS1OdometryVisualizer(String title, String topic)
   {
      super(title);
      this.topic = topic;
   }

   public void setFrame(ReferenceFrame frame)
   {
      this.frame = frame;
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<PoseStamped>(PoseStamped._TYPE)
      {
         @Override
         public void onNewMessage(PoseStamped pose)
         {
            RDXROS1OdometryVisualizer.this.pose = pose;
            frequencyPlot.recordEvent();
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
         if (frame != null)
            framePose.setToZero(frame);
         else
            framePose.setToZero(ReferenceFrame.getWorldFrame());

         RosTools.toEuclid(pose.getPose(), framePose);
         framePose.changeFrame(ReferenceFrame.getWorldFrame());

         modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.1);
         LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);
         poseModels.clear();
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
      frequencyPlot.renderImGuiWidgets();
   }

   @Override
   public void update()
   {
      super.update();
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
