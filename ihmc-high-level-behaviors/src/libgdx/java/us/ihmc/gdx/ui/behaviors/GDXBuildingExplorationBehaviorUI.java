package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.vr.*;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.behaviors.demo.BuildingExplorationStateName;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Trigger;

public class GDXBuildingExplorationBehaviorUI implements RenderableProvider
{
   private static final String WINDOW_NAME = "Building Exploration";

   private Messager messager;
   private final FramePose3D tempFramePose = new FramePose3D();
   private ModelInstance goalSphere;
   private boolean goalIsBeingPlaced;

   public void create(GDXVRManager vrManager)
   {
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
                     messager.submitMessage(BuildingExplorationBehaviorAPI.RequestedState, BuildingExplorationStateName.LOOK_AND_STEP);
                     messager.submitMessage(BuildingExplorationBehaviorAPI.Start, true);
                  }
               }
               if (device == vrManager.getControllers().get(RobotSide.RIGHT))
               {
                  if (button == GDXVRControllerButtons.Grip)
                  {
                     messager.submitMessage(BuildingExplorationBehaviorAPI.Stop, true);
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

   public void handleVREvents(GDXVRManager vrManager)
   {
      if (goalIsBeingPlaced)
      {
         vrManager.getControllers().get(RobotSide.LEFT).getPose(ReferenceFrame.getWorldFrame(), goalSphere.transform);
         messager.submitMessage(BuildingExplorationBehaviorAPI.Goal, new Pose3D(tempFramePose));
      }
   }

   public void renderImGuiWindows()
   {
      ImGui.begin(WINDOW_NAME);

      ImGui.button("Place goal");
//      ImGui.dragFlo

      ImGui.end();
   }

   public void setMessager(Messager messager)
   {
      this.messager = messager;
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
