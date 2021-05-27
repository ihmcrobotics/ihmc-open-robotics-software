package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.interfaces.MessagerPublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.vr.*;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.behaviors.demo.BuildingExplorationStateName;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Trigger;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   private static final String WINDOW_NAME = "Building Exploration";

   private final MessagerPublishSubscribeAPI messager;
   private final FramePose3D tempFramePose = new FramePose3D();
   private ModelInstance goalSphere;
   private boolean goalIsBeingPlaced;

   public ImGuiGDXBuildingExplorationBehaviorUI(MessagerPublishSubscribeAPI messager)
   {
      this.messager = messager;
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
                     messager.publish(BuildingExplorationBehaviorAPI.RequestedState, BuildingExplorationStateName.LOOK_AND_STEP);
                     messager.publish(BuildingExplorationBehaviorAPI.Start, true);
                  }
               }
               if (device == vrManager.getControllers().get(RobotSide.RIGHT))
               {
                  if (button == GDXVRControllerButtons.Grip)
                  {
                     messager.publish(BuildingExplorationBehaviorAPI.Stop, true);
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
   }

   @Override
   public void handleVREvents(GDXVRManager vrManager)
   {
      if (goalIsBeingPlaced)
      {
         vrManager.getControllers().get(RobotSide.LEFT).getPose(ReferenceFrame.getWorldFrame(), goalSphere.transform);
         messager.publish(BuildingExplorationBehaviorAPI.Goal, new Pose3D(tempFramePose));
      }
   }

   public void renderImGuiWindows()
   {
      ImGui.begin(WINDOW_NAME);

      ImGui.end();
   }

   @Override
   public void render()
   {
      ImGui.button("Place goal");
   }

   @Override
   public void destroy()
   {

   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalSphere.getRenderables(renderables, pool);
   }
}
