package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehavior;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorAPI.*;

public class ImGuiGDXTargetFollowingBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(TargetFollowingBehavior.DEFINITION,
                                                                                                  ImGuiGDXTargetFollowingBehaviorUI::new);
   private final BehaviorHelper helper;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final TargetFollowingBehaviorParameters targetFollowingParameters = new TargetFollowingBehaviorParameters();
   private final ImGuiStoredPropertySetTuner targetFollowingParameterTuner = new ImGuiStoredPropertySetTuner("Target Following Parameters");
   private final ImGuiGDXPoseGoalAffordance manualTargetAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private int pointNumber;
   private final FramePose3D targetPose = new FramePose3D();
   private final FramePose3D robotMidFeetUnderPelvisPose = new FramePose3D();
   private final PausablePeriodicThread periodicThread;
   private final AtomicReference<TimeStampedTransform3D> latestSemanticTargetPose = new AtomicReference<>();
   private ModelInstance targetApproachPoseGraphic;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private AtomicReference<Pose3D> targetApproachPoseReference;

   public ImGuiGDXTargetFollowingBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);

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
            lookAndStepUI.setGoal(targetPose);
         }

         ++pointNumber;
      });
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      targetFollowingParameterTuner.create(targetFollowingParameters,
                                           TargetFollowingBehaviorParameters.keys,
                                           () -> helper.publish(TargetFollowingParameters, targetFollowingParameters.getAllAsStrings()));
      manualTargetAffordance.create(baseUI, placedTargetPose ->
      {
         TimeStampedTransform3D targetTransform3D = new TimeStampedTransform3D();
         targetTransform3D.setTransform3D(placedTargetPose);
         targetTransform3D.setTimeStamp(System.nanoTime());
         latestSemanticTargetPose.set(targetTransform3D);
      }, Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(manualTargetAffordance::processImGui3DViewInput);
      lookAndStepUI.create(baseUI);

      targetApproachPoseGraphic = GDXModelPrimitives.createCoordinateFrameInstance(0.1);
      targetApproachPoseReference = helper.subscribeViaReference(TargetApproachPose, BehaviorTools.createNaNPose());
   }

   @Override
   public void update()
   {
      // periodicThread.setRunning(wasTickedRecently(0.5));
      GDXTools.toGDX(targetApproachPoseReference.get(), tempTransform, targetApproachPoseGraphic.transform);

      lookAndStepUI.update();
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      manualTargetAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
      targetFollowingParameterTuner.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         targetApproachPoseGraphic.getRenderables(renderables, pool);
      }
      manualTargetAffordance.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }

   private boolean areGraphicsEnabled()
   {
      return wasTickedRecently(0.5);
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      periodicThread.destroy();
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
