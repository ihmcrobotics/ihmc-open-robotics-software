package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import perception_msgs.msg.dds.BallDetectionParametersMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXROS2BallTrackingVisualizer extends RDXVisualizer implements ROS2TopicHolder<RigidBodyTransformMessage>
{
   private static final double MESSAGE_PUBLISH_PERIOD = 2; // publish messages every 2 seconds

   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<RigidBodyTransformMessage> ballPositionTopic;
   private RealtimeROS2Node realtimeROS2Node;
   private final ROS2PublishSubscribeAPI ros2;

   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final ConcurrentLinkedQueue<RigidBodyTransform> trajectories = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Instant> trajectoryAcquisitionTimes = new ConcurrentLinkedQueue<>();

   // Visualizer settings
   private final ImGuiExpandCollapseRenderer visualizerSettingsCollapser = new ImGuiExpandCollapseRenderer();
   private boolean showVisualizerSettings = false;
   private final ImInt historySpan = new ImInt(2);
   private final ImFloat lineWidth = new ImFloat(0.015f);

   // Ball tracking settings
   private final ImGuiExpandCollapseRenderer ballTrackingSettingsCollapser = new ImGuiExpandCollapseRenderer();
   private boolean showBallTrackingSettings = false;

   private final ImFloat ballDiameter = new ImFloat(0.0398f);
   private final ImFloat positionAlphaFilter = new ImFloat(0.2f);

   private final ImFloat hLowerBound = new ImFloat(40.0f);
   private final ImFloat sLowerBound = new ImFloat(75.0f);
   private final ImFloat vLowerBound = new ImFloat(80.0f);

   private final ImFloat hUpperBound = new ImFloat(85.0f);
   private final ImFloat sUpperBound = new ImFloat(255.0f);
   private final ImFloat vUpperBound = new ImFloat(255.0f);

   private final Notification trackingParametersChanged = new Notification();
   private final Throttler messagePublishThrottler = new Throttler().setPeriod(MESSAGE_PUBLISH_PERIOD);

   public RDXROS2BallTrackingVisualizer(String title, ROS2Topic<RigidBodyTransformMessage> ballPositionTopic, ROS2PublishSubscribeAPI ros2PubSubAPI)
   {
      this(title, PubSubImplementation.FAST_RTPS, ballPositionTopic, ros2PubSubAPI);
   }

   public RDXROS2BallTrackingVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<RigidBodyTransformMessage> ballPositionTopic, ROS2PublishSubscribeAPI ros2PubSubAPI)
   {
      super(title);

      this.titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.ballPositionTopic = ballPositionTopic;
      this.ros2 = ros2PubSubAPI;

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && realtimeROS2Node == null)
            subscribe();
         else if (!isActive && realtimeROS2Node != null)
            unsubscribe();
      });
   }

   private void subscribe()
   {
      // if already subscribed, unsubscribe
      if (realtimeROS2Node != null)
         unsubscribe();

      // subscribe
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      realtimeROS2Node.createSubscription(ballPositionTopic, this::updateGraphic);
      realtimeROS2Node.spin();
   }

   private void unsubscribe()
   {
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
      trajectories.clear();
   }

   private void updateGraphic(Subscriber<RigidBodyTransformMessage> subscriber)
   {
      if (subscriber.isAvailable())
      {
         RigidBodyTransform newTransform = MessageTools.toEuclid(subscriber.takeNextData());
         trajectories.offer(newTransform);
         trajectoryAcquisitionTimes.offer(Instant.now());
      }
   }

   @Override
   public void update()
   {
      super.update();

      // Remove old trajectories over the history length and add new trajectory
      Instant now = Instant.now();
      while (trajectoryAcquisitionTimes.peek() != null && hasExpired(now, trajectoryAcquisitionTimes.peek(), historySpan.get()))
      {
         trajectories.poll();
         trajectoryAcquisitionTimes.poll();
      }

      trajectoryGraphic.update(lineWidth.get(), trajectories);

      if (trackingParametersChanged.poll() || messagePublishThrottler.run())
      {
         BallDetectionParametersMessage message = new BallDetectionParametersMessage();
         message.setBallDiameter(ballDiameter.get());
         message.setAlpha(positionAlphaFilter.get());

         message.setHLow(hLowerBound.get());
         message.setSLow(sLowerBound.get());
         message.setVLow(vLowerBound.get());

         message.setHHigh(hUpperBound.get());
         message.setSHigh(sUpperBound.get());
         message.setVHigh(vUpperBound.get());

         ros2.publish(PerceptionAPI.BALL_DETECTION_PARAMETERS, message);
      }
   }

   private boolean hasExpired(Instant now, Instant old, long lifespan)
   {
      return old.plusSeconds(lifespan).isBefore(now);
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (visualizerSettingsCollapser.render(showVisualizerSettings))
         showVisualizerSettings = !showVisualizerSettings;
      ImGui.sameLine();
      ImGui.text("Visualizer Settings");
      if (showVisualizerSettings)
      {
         ImGuiTools.volatileInputInt("History (seconds)", historySpan, 1);
         ImGuiTools.volatileInputFloat("Line Width", lineWidth);
      }

      if (ballTrackingSettingsCollapser.render(showBallTrackingSettings))
         showBallTrackingSettings = !showBallTrackingSettings;
      ImGui.sameLine();
      ImGui.text("Ball Tracking Settings");
      if (showBallTrackingSettings)
      {
         if (ImGuiTools.volatileInputFloat("Ball Diameter", ballDiameter))
            trackingParametersChanged.set();
         if (ImGui.sliderFloat("Position Alpha Filter", positionAlphaFilter.getData(), 0.0f, 1.0f))
            trackingParametersChanged.set();

         ImGui.text("HSV Lower Bound");
         if (ImGui.sliderFloat("Hl", hLowerBound.getData(), 0.0f, 179.0f))
            trackingParametersChanged.set();
         if (ImGui.sliderFloat("Sl", sLowerBound.getData(), 0.0f, 255.0f))
            trackingParametersChanged.set();
         if (ImGui.sliderFloat("Vl", vLowerBound.getData(), 0.0f, 255.0f))
            trackingParametersChanged.set();

         ImGui.text("HSV Upper Bound");
         if (ImGui.sliderFloat("Hu", hUpperBound.getData(), 0.0f, 179.0f))
            trackingParametersChanged.set();
         if (ImGui.sliderFloat("Su", sUpperBound.getData(), 0.0f, 255.0f))
            trackingParametersChanged.set();
         if (ImGui.sliderFloat("Vu", vUpperBound.getData(), 0.0f, 255.0f))
            trackingParametersChanged.set();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      trajectoryGraphic.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }

   @Override
   public ROS2Topic<RigidBodyTransformMessage> getTopic()
   {
      return ballPositionTopic;
   }
}
