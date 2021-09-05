package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehavior;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.RosPoseStampedSubscriber;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXTargetFollowingBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(TargetFollowingBehavior.DEFINITION,
                                                                                                  ImGuiGDXTargetFollowingBehaviorUI::new);

   private final BehaviorHelper helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private TargetFollowingBehaviorParameters parameters;
   private final ImGuiStoredPropertySetTuner parameterTuner = new ImGuiStoredPropertySetTuner("Target Following Parameters");
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private String lastTickedThing = "NONE";
   private int pointNumber;
   private final FramePose3D goalPose = new FramePose3D();
   private final PausablePeriodicThread periodicThread;
   private final AtomicReference<TimeStampedTransform3D> latestSemanticTargetPose = new AtomicReference<>();

   public ImGuiGDXTargetFollowingBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      syncedRobot = helper.newSyncedRobot();
      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);
      helper.subscribeToPlanarRegionsViaCallback(ROS2Tools.MAPSENSE_REGIONS, regions ->
      {
         goalAffordance.setLatestRegions(regions);
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      helper.subscribeViaCallback(LastTickedThing, lastTickedThing -> this.lastTickedThing = lastTickedThing);
      helper.getROS1Helper().attachSubscriber(RosTools.SEMANTIC_TARGET_POSE, new RosPoseStampedSubscriber()
      {
         @Override
         protected void newPose(String frameID, TimeStampedTransform3D transform)
         {
            synchronized (this)
            {
               latestSemanticTargetPose.set(transform);
            }
         }
      });

      pointNumber = 0;
      int numberOfPoints = 20;
      double radius = 3.0;
      periodicThread = new PausablePeriodicThread("GoalProducerThread", 2.0, () ->
      {
         if (pointNumber >= numberOfPoints)
         {
            pointNumber = 0;
         }

         double percentage2PI = (pointNumber / (double) numberOfPoints) * 2.0 * Math.PI;
         double x = radius * Math.cos(percentage2PI);
         double y = radius * Math.sin(percentage2PI);
         double yaw = Math.PI / 2.0 + percentage2PI;
//         goalPose.set(x, y, 0.0, yaw, 0.0, 0.0);
         synchronized (this)
         {
            lookAndStepUI.setGoal(goalPose);
         }

         ++pointNumber;
      });
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      parameters = new TargetFollowingBehaviorParameters();
      parameterTuner.create(parameters, TargetFollowingBehaviorParameters.keys, () -> helper.publish(Parameters, parameters.getAllAsStrings()));
      goalAffordance.create(baseUI, goalPose ->
      {
         lookAndStepUI.setGoal(goalPose);
      }, Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);
      lookAndStepUI.create(baseUI);
   }

   @Override
   public void update()
   {
      // TODO: Need to get the state of the remote node. Is it actively being ticked?
      periodicThread.setRunning(wasTickedRecently(0.5));

      if (latestSemanticTargetPose.get() != null)
      {
         syncedRobot.update();
         goalPose.setIncludingFrame(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame(), latestSemanticTargetPose.get().getTransform3D());
         goalPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalPose.getPosition().setZ(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ());
      }

      if (areGraphicsEnabled())
      {
         planarRegionsGraphic.update();
      }
      lookAndStepUI.update();
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      goalAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
      parameterTuner.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
      goalAffordance.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }

   private boolean areGraphicsEnabled()
   {
      return wasTickedRecently(0.5) && lastTickedThing.equals("NONE");
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      periodicThread.destroy();
      planarRegionsGraphic.destroy();
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
