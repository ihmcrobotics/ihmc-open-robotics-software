package us.ihmc.rdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import geometry_msgs.PoseStamped;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXTargetFollowingBehaviorUI extends RDXBehaviorUIInterface
{
   private final BehaviorHelper helper;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean publishTestLoop = new ImBoolean(false);
   private final TargetFollowingBehaviorParameters targetFollowingParameters = new TargetFollowingBehaviorParameters();
   private final RDXStoredPropertySetTuner targetFollowingParameterTuner = new RDXStoredPropertySetTuner("Target Following Parameters");
   private final RDXBallAndArrowPosePlacement manualTargetAffordance = new RDXBallAndArrowPosePlacement();
   private final RosTopicPublisher<PoseStamped> manualTargetPublisher = null;
   private int pointNumber;
   private final FramePose3D testLoopTargetPose = new FramePose3D();
   private final FramePose3D manualTargetPose = new FramePose3D();
   private final PausablePeriodicThread periodicThread;
   private ModelInstance targetApproachPoseGraphic;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private AtomicReference<Pose3D> targetApproachPoseReference;
   private final RDXLookAndStepBehaviorUI lookAndStepUI;
   private final AtomicReference<Pose3D> latestTargetPoseFromBehaviorReference = new AtomicReference<>();
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXTargetFollowingBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      syncedRobot = helper.newSyncedRobot();
      lookAndStepUI = new RDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);

//      manualTargetPublisher = helper.getROS1Helper().publishPose(RosTools.SEMANTIC_TARGET_POSE);
//      helper.subscribeViaCallback(TargetPose, latestTargetPoseFromBehaviorReference::set);

      pointNumber = 0;
      int numberOfPoints = 20;
      periodicThread = new PausablePeriodicThread("GoalProducerThread", 2.0, () ->
      {
         double radius = targetFollowingParameters.getTestLoopRadius();
         if (pointNumber >= numberOfPoints)
            pointNumber = 0;
         double percentage2PI = (pointNumber / (double) numberOfPoints) * 2.0 * Math.PI;
         double x = radius * Math.cos(percentage2PI);
         double y = radius * Math.sin(percentage2PI);
         double yaw = Math.PI / 2.0 + percentage2PI;
         testLoopTargetPose.setToZero(ReferenceFrame.getWorldFrame());
         testLoopTargetPose.set(x, y, 0.0, yaw, 0.0, 0.0);

         syncedRobot.update();
         testLoopTargetPose.changeFrame(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());

         PoseStamped poseStampedMessage = manualTargetPublisher.getMessage();
         RosTools.toRos(testLoopTargetPose, poseStampedMessage.getPose());
         manualTargetPublisher.publish(poseStampedMessage);
         ++pointNumber;
      });
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {
      targetFollowingParameterTuner.create(targetFollowingParameters,
                                           () ->
                                           {
//                                              helper.publish(TargetFollowingParameters, targetFollowingParameters.getAllAsStrings());
                                           });
      targetApproachPoseGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.1);
//      targetApproachPoseReference = helper.subscribeViaReference(TargetApproachPose, BehaviorTools.createNaNPose());
      manualTargetAffordance.create(placedTargetPose ->
      {
         syncedRobot.update();
         manualTargetPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), placedTargetPose);
         manualTargetPose.changeFrame(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());

         PoseStamped poseStampedMessage = manualTargetPublisher.getMessage();
         RosTools.toRos(manualTargetPose, poseStampedMessage.getPose());
         manualTargetPublisher.publish(poseStampedMessage);
      }, Color.GREEN);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualTargetAffordance::processImGui3DViewInput);

      lookAndStepUI.create(baseUI);
   }

   @Override
   public void update()
   {
      if (publishTestLoop.get())
         periodicThread.setRunning(getState().getIsActive());
      else
         periodicThread.setRunning(false);

      Pose3D latestTargetPoseFromBehavior = latestTargetPoseFromBehaviorReference.getAndSet(null);
      if (latestTargetPoseFromBehavior != null)
      {
         manualTargetAffordance.setGoalPoseNoCallbacks(latestTargetPoseFromBehavior);
      }

      LibGDXTools.toLibGDX(targetApproachPoseReference.get(), tempTransform, targetApproachPoseGraphic.transform);
      lookAndStepUI.update();
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      ImGui.text("Goal Planning");
      ImGui.sameLine();
      manualTargetAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
      ImGui.checkbox(labels.get("Publish test loop"), publishTestLoop);
      targetFollowingParameterTuner.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (areGraphicsEnabled())
         {
            targetApproachPoseGraphic.getRenderables(renderables, pool);
         }
         manualTargetAffordance.getRenderables(renderables, pool);
      }
      lookAndStepUI.getRenderables(renderables, pool, sceneLevels);
   }

   private boolean areGraphicsEnabled()
   {
      return getState().getIsActive();
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      periodicThread.destroy();
   }

   public String getName()
   {
      return "Target Following";
   }
}
