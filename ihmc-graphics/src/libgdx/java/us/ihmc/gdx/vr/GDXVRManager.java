package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.function.Consumer;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Touchpad;

public class GDXVRManager
{
   private final GDXVRContext context = new GDXVRContext();
   private Notification contextCreatedNotification;
   private boolean contextInitialized = false;
   private boolean initializing = false;
   private boolean skipHeadset = false;
   private boolean holdingTouchpadToMove = false;
   private final Point3D initialVRSpacePosition = new Point3D();
   private final Point3D initialVRControllerPosition = new Point3D();
   private final Point3D currentVRControllerPosition = new Point3D();
   private final Vector3D deltaVRControllerPosition = new Vector3D();
   private final Point3D resultVRSpacePosition = new Point3D();
   private final Point3D lastVRSpacePosition = new Point3D();
   private final ImBoolean vrEnabled = new ImBoolean(false);
   private final ArrayList<Consumer<GDXVRManager>> vrInputProcessors = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   /**
    * Doing poll and render close together makes VR performance the best it can be
    * and reduce dizziness.
    */
   public void pollEventsAndRender(GDXImGuiBasedUI baseUI, GDX3DSceneManager sceneManager)
   {
      boolean posesReady = pollEvents(baseUI);
      if (posesReady && isVRReady())
      {
         skipHeadset = true;
         context.renderEyes(sceneManager);
         skipHeadset = false;
      }
   }

   private boolean pollEvents(GDXImGuiBasedUI baseUI)
   {
      boolean posesReady = false;
      if (vrEnabled.get())
      {
         if (!initializing && contextCreatedNotification == null) // should completely dispose and recreate?
         {
            initializing = true;
            contextCreatedNotification = context.initSystemOnAThread();
         }
         if (contextCreatedNotification != null && contextCreatedNotification.poll())
         {
            initializing = false;
            context.setupEyes();

            if (!Boolean.parseBoolean(System.getProperty("gdx.free.spin")))
            {
               baseUI.setForegroundFPS(240);
            }
            baseUI.setVsync(false); // important to disable vsync for VR


            contextInitialized = true;
         }

         if (contextInitialized)
         {
            posesReady = context.pollEvents();

            if (posesReady)
            {
               // TODO: Implement VR teleport control here; extract teleportation manager
               context.getController(RobotSide.RIGHT, controller ->
               {
                  holdingTouchpadToMove = controller.isButtonPressed(SteamVR_Touchpad);
               });
               //               GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), initialVRControllerPosition);
               //               GDXTools.toEuclid(context.getTrackerSpaceOriginToWorldSpaceTranslationOffset(), initialVRSpacePosition);
               //               context.getToZUpXForward().transform(initialVRSpacePosition);
               //               lastVRSpacePosition.set(initialVRSpacePosition);
               if (context.isHeadsetConnected())
               {
                  context.teleport(scenePoseGizmo.getTransform());

                  if (holdingTouchpadToMove)
                  {
                     //         GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), currentVRControllerPosition);
                     //         resultVRSpacePosition.set(initialVRSpacePosition);
                     //         deltaVRControllerPosition.sub(currentVRControllerPosition, initialVRControllerPosition);
                     //         resultVRSpacePosition.sub(deltaVRControllerPosition);
                     //         resultVRSpacePosition.sub(lastVRSpacePosition);
                     //         GDXTools.toGDX(resultVRSpacePosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
                     //         lastVRSpacePosition.set(resultVRSpacePosition);
                     //         context.getToZUpXForward().inverseTransform(currentVRControllerPosition);
                     //         GDXTools.toGDX(currentVRControllerPosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
                     //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.1f, 0.0f, 0.0f);
                  }
                  else
                  {
                     //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.0f, 0.0f, 0.0f);
                  }
               }

               for (Consumer<GDXVRManager> vrInputProcessor : vrInputProcessors)
               {
                  vrInputProcessor.accept(this);
               }
            }
         }
      }
      else
      {
         if (contextCreatedNotification != null && contextInitialized)
         {
            dispose();
         }
      }
      return posesReady;
   }

   public void renderImGuiEnableWidget()
   {
      if (ImGui.checkbox(labels.get("VR Enabled"), vrEnabled))
      {
         if (vrEnabled.get())
            LogTools.info("Enabling VR");
         else
            LogTools.info("Disabling VR");
      }
      if (ImGui.isItemHovered())
      {
         float right = ImGui.getWindowPosX() + ImGui.getWindowSizeX();
         float y = ImGui.getItemRectMaxY();
         ImGui.setNextWindowPos(right - 600, y); // prevent the tooltip from creating a new window
         ImGui.setTooltip("It is recommended to start SteamVR and power on the VR controllers before clicking this button.");
      }
   }

   public void dispose()
   {
      if (contextCreatedNotification != null && contextInitialized)
      {
         contextCreatedNotification = null;
         contextInitialized = false;
         context.dispose();
      }
   }

   public boolean isVRReady()
   {
      // Wait for VR setup to be ready. This is the primary indicator, called only when the headset is connected
      return vrEnabled.get() && contextInitialized && context.isHeadsetConnected();
   }

   public void addVRInputProcessor(Consumer<GDXVRManager> processVRInput)
   {
      vrInputProcessors.add(processVRInput);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (isVRReady())
      {
         scenePoseGizmo.process3DViewInput(input);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (vrEnabled.get() && contextInitialized)
      {
         if (!skipHeadset)
         {
            context.getHeadset(headset -> headset.getModelInstance().getRenderables(renderables, pool));
            context.getHeadsetCoordinateFrame().getRenderables(renderables, pool);
         }
         for (RobotSide side : RobotSide.values)
         {
            context.getController(side, controller -> controller.getModelInstance().getRenderables(renderables, pool));
            context.getEyes().get(side).getCoordinateFrameInstance().getRenderables(renderables, pool);
         }
         context.getBaseStations(baseStation ->
         {
            baseStation.getModelInstance().getRenderables(renderables, pool);
         });
         context.getGenericDevices(genericDevice ->
         {
            genericDevice.getModelInstance().getRenderables(renderables, pool);
         });

         scenePoseGizmo.getRenderables(renderables, pool);
      }
   }

   public GDXVRContext getContext()
   {
      return context;
   }

   public ImBoolean getVREnabled()
   {
      return vrEnabled;
   }
}
