package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.internal.ImGui;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.vr.*;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.behaviors.demo.BuildingExplorationStateName;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Trigger;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final FramePose3D tempFramePose = new FramePose3D();
   private ModelInstance goalSphere;
   private boolean goalIsBeingPlaced;
   private final GDXFootstepPlanGraphic controllerFootsteps = new GDXFootstepPlanGraphic();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      helper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         controllerFootsteps.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps));
      });

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      GDXVRManager vrManager = baseUI.getVRManager();
      goalSphere = GDXModelPrimitives.createSphere(0.1f, Color.GREEN);

      if (GDXVRManager.isVREnabled())
      {
         vrManager.getContext().addListener(new GDXVRDeviceAdapter()
         {
            @Override
            public void buttonPressed(GDXVRDevice device, int button)
            {
               LogTools.info("Pressed: {}, {}", device, button);
               if (device == vrManager.getControllers().get(RobotSide.LEFT))
               {
                  if (button == SteamVR_Trigger)
                  {
                     goalIsBeingPlaced = true;
                  }
                  else if (button == GDXVRControllerButtons.Grip)
                  {
                     helper.publish(BuildingExplorationBehaviorAPI.RequestedState, BuildingExplorationStateName.LOOK_AND_STEP);
                     helper.publish(BuildingExplorationBehaviorAPI.Start, true);
                  }
               }
               if (device == vrManager.getControllers().get(RobotSide.RIGHT))
               {
                  if (button == GDXVRControllerButtons.Grip)
                  {
                     helper.publish(BuildingExplorationBehaviorAPI.Stop, true);
                  }
               }
            }

            @Override
            public void buttonReleased(GDXVRDevice device, int button)
            {
               LogTools.info("Released: {}, {}", device, button);
               if (device == vrManager.getControllers().get(RobotSide.LEFT) && button == SteamVR_Trigger)
               {
                  goalIsBeingPlaced = false;
               }
            }
         });
      }

      lookAndStepUI.create(baseUI);
   }

   @Override
   public void handleVREvents(GDXVRManager vrManager)
   {
      if (goalIsBeingPlaced)
      {
         vrManager.getControllers().get(RobotSide.LEFT).getPose(ReferenceFrame.getWorldFrame(), goalSphere.transform);
         helper.publish(BuildingExplorationBehaviorAPI.Goal, new Pose3D(tempFramePose));
      }
   }

   @Override
   public void render()
   {
      ImGui.button("Place goal");

      int defaultOpen = ImGuiTreeNodeFlags.DefaultOpen;
      if (ImGui.collapsingHeader("Look and Step", defaultOpen))
      {
         lookAndStepUI.render();
      }

      controllerFootsteps.render();
   }

   @Override
   public void destroy()
   {
      controllerFootsteps.destroy();
      lookAndStepUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalSphere.getRenderables(renderables, pool);
      controllerFootsteps.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }
}
