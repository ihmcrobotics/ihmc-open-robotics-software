package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import geometry_msgs.PoseStamped;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2RigidBodyPoseVisualizer extends RDXVisualizer implements RenderableProvider
{
   private ModelInstance poseModel;
   private ReferenceFrame frame;
   private PoseStamped pose;
   private final FramePose3D framePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ROS2Topic<Pose3D> topic;
   private final AtomicReference<Pose3D> transformMessageReference = new AtomicReference<>();
   private final SampleInfo sampleInfo = new SampleInfo();

   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot numberOfRegionsPlot = new ImGuiPlot("# Regions", 1000, 230, 20);
   private int numberOfPlanarRegions = 0;

   private RealtimeROS2Node realtimeROS2Node;
   private final String titleBeforeAdditions;
   private final DomainFactory.PubSubImplementation pubSubImplementation;
   private final ImBoolean subscribed = new ImBoolean(false);
   private final Object syncObject = new Object();
   private final Pose3D message = new Pose3D();

   public RDXROS2RigidBodyPoseVisualizer(String title, DomainFactory.PubSubImplementation pubSubImplementation, ROS2Topic<Pose3D> topic)
   {
      super(title + " (ROS 2)");
      titleBeforeAdditions = title;
      this.topic = topic;
      this.pubSubImplementation = pubSubImplementation;
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      frequencyPlot.renderImGuiWidgets();
      numberOfRegionsPlot.render(numberOfPlanarRegions);
   }

   @Override
   public void update()
   {
      super.update();

      Pose3D transformMessage = transformMessageReference.getAndSet(null);

      if(transformMessage != null)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
//         MessageTools.toEuclid(message, transform);
         this.framePose.changeFrame(ReferenceFrame.getWorldFrame());
         poseModel = RDXModelBuilder.createCoordinateFrameInstance(0.1);
         LibGDXTools.toLibGDX(this.framePose, this.tempTransform, poseModel.transform);
      }
   }

   public void queueRenderRigidBodyPose(Subscriber<Pose3D> subscriber)
   {
      synchronized (syncObject)
      {
         subscriber.takeNextData(message, sampleInfo);

         transformMessageReference.set(message);

      }

   }

   private void subscribe()
   {
      subscribed.set(true);
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::queueRenderRigidBodyPose);
      realtimeROS2Node.spin();
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && realtimeROS2Node == null)
      {
         subscribe();
      }
      else if (!subscribed && realtimeROS2Node != null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (poseModel != null)
      {
         poseModel.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
   }
}
