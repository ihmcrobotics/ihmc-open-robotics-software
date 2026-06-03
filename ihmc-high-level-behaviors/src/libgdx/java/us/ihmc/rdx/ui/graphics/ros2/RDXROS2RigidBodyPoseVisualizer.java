package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.RigidBodyTransformMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.tools.string.StringTools;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2RigidBodyPoseVisualizer extends RDXROS2SingleTopicVisualizer<RigidBodyTransformMessage>
{
   private ModelInstance poseModel;
   private ReferenceFrame frame;
   private final FramePose3D framePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ROS2Topic<RigidBodyTransformMessage> topic;
   private final AtomicReference<Pose3D> transformMessageReference = new AtomicReference<>();

   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot numberOfRegionsPlot = new ImGuiPlot("# Regions", 1000, 230, 20);
   private int numberOfPlanarRegions = 0;

   private ROS2Node ros2Node;
   private final String titleBeforeAdditions;

   public RDXROS2RigidBodyPoseVisualizer(String title, ROS2Topic<RigidBodyTransformMessage> topic)
   {
      super(title);
      titleBeforeAdditions = title;
      this.topic = topic;

      addActivenessChangeCallback(isActive ->
                                  {
                                     if (isActive && ros2Node == null)
                                     {
                                        subscribe();
                                     }
                                     else if (!isActive && ros2Node != null)
                                     {
                                        unsubscribe();
                                     }
                                  });
   }

   @Override
   public void renderImGuiWidgets()
   {
      frequencyPlot.renderImGuiWidgets();
      numberOfRegionsPlot.render(numberOfPlanarRegions);
   }

   @Override
   public void update()
   {
      super.update();

      Pose3D transformMessage = transformMessageReference.getAndSet(null);

      if (transformMessage != null)
      {
         this.framePose.changeFrame(ReferenceFrame.getWorldFrame());
         poseModel = RDXModelBuilder.createCoordinateFrameInstance(0.1);
         LibGDXTools.toLibGDX(this.framePose, this.tempTransform, poseModel.transform);

         getFrequency().ping();
      }
   }

   public void queueRenderRigidBodyPose(RigidBodyTransformMessage message)
   {
      if (message == null)
         return;
      RigidBodyTransform transform = MessageTools.toEuclid(message);
      Pose3D pose = new Pose3D(transform);
      transformMessageReference.set(pose);
   }

   private void subscribe()
   {
      ros2Node = new ROS2Node(StringTools.titleToSnakeCase(titleBeforeAdditions));

      ros2Node.createSubscriptionSampler(topic, this::queueRenderRigidBodyPose);
   }

   private void unsubscribe()
   {
      if (ros2Node != null)
      {
         ros2Node.close();
         ros2Node = null;
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (poseModel != null)
      {
         poseModel.getRenderables(renderables, pool);
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
   }

   @Override
   public ROS2Topic<RigidBodyTransformMessage> getTopic()
   {
      return topic;
   }
}
