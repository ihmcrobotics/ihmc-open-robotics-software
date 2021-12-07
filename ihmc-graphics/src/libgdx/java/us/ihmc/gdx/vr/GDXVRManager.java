package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.exception.DefaultExceptionHandler;
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
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.RejectedExecutionException;

/** This class should manage VR as part of the ImGuiBasedUI. */
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
   private final GDXPose3DGizmo scenePoseGizmo = new GDXPose3DGizmo();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean vrEnabled = new ImBoolean(false);
   private final ResettableExceptionHandlingExecutorService waitGetPosesExecutor = MissingThreadTools.newSingleThreadExecutor("PoseWaiterOnner", true, 1);
   private final Notification posesReady = new Notification();

   public GDXVRManager()
   {
      context.addVRInputProcessor(context ->
      {
         // TODO: Implement VR teleport control here; extract teleportation manager
//         context.getController(RobotSide.RIGHT, controller ->
//         {
//            holdingTouchpadToMove = controller.isButtonPressed(SteamVR_Touchpad);
//         });
//         //               GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), initialVRControllerPosition);
//         //               GDXTools.toEuclid(context.getTrackerSpaceOriginToWorldSpaceTranslationOffset(), initialVRSpacePosition);
//         //               context.getToZUpXForward().transform(initialVRSpacePosition);
//         //               lastVRSpacePosition.set(initialVRSpacePosition);
//         if (context.isHeadsetConnected())
//         {
//            context.teleport(scenePoseGizmo.getTransform());
//
//            if (holdingTouchpadToMove)
//            {
//               //         GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), currentVRControllerPosition);
//               //         resultVRSpacePosition.set(initialVRSpacePosition);
//               //         deltaVRControllerPosition.sub(currentVRControllerPosition, initialVRControllerPosition);
//               //         resultVRSpacePosition.sub(deltaVRControllerPosition);
//               //         resultVRSpacePosition.sub(lastVRSpacePosition);
//               //         GDXTools.toGDX(resultVRSpacePosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
//               //         lastVRSpacePosition.set(resultVRSpacePosition);
//               //         context.getToZUpXForward().inverseTransform(currentVRControllerPosition);
//               //         GDXTools.toGDX(currentVRControllerPosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
//               //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.1f, 0.0f, 0.0f);
//            }
//            else
//            {
//               //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.0f, 0.0f, 0.0f);
//            }
//         }
      });
   }

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
         context.renderEyes(sceneManager.getSceneBasics());
         skipHeadset = false;
      }
   }

   private boolean pollEvents(GDXImGuiBasedUI baseUI)
   {
      boolean posesReadyThisFrame = false;
      if (vrEnabled.get())
      {
         if (!initializing && contextCreatedNotification == null) // should completely dispose and recreate?
         {
            initializing = true;
            contextCreatedNotification = new Notification();
            MissingThreadTools.startAsDaemon(getClass().getSimpleName() + "-initSystem", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
            {
               context.initSystem();
               contextCreatedNotification.set();
            });
         }
         if (contextCreatedNotification != null && contextCreatedNotification.poll())
         {
            initializing = false;
            context.setupEyes();

            if (!Boolean.parseBoolean(System.getProperty("gdx.free.spin")))
            {
               baseUI.setForegroundFPS(240); // TODO: Do something better with this
            }
            baseUI.setVsync(false); // important to disable vsync for VR

            scenePoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());

            contextInitialized = true;
         }

         if (contextInitialized)
         {
            // Waiting for the poses on a thread allows for the rest of the application to keep
            // cranking faster than VR can do stuff. This also makes the performance graph in Steam
            // show the correct value and the OpenVR stack work much better.
            try
            {
               waitGetPosesExecutor.clearQueueAndExecute(() ->
               {
                  context.waitGetPoses();
                  posesReady.set();
               });

               // TODO: This whole thing might have major issues because
               // there's a delay waiting for the next time this method is called
               posesReadyThisFrame = posesReady.poll();
            }
            catch (RejectedExecutionException rejectedExecutionException)
            {
               // TODO: If this happens a lot but is fine, maybe it should be built into ResettableExceptionHandlingExecutorService
               LogTools.info("Resetting the WaitGetPoses executor.");
               waitGetPosesExecutor.interruptAndReset();
            }

            if (posesReadyThisFrame)
            {
               context.pollEvents(); // FIXME: Potential bug is that the poses get updated in the above thread while they're being used in here
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

      return posesReadyThisFrame;
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
      waitGetPosesExecutor.destroy();
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
      return vrEnabled.get() && contextInitialized && context.getHeadset().isConnected();
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
            context.getHeadset().runIfConnected(headset -> headset.getModelInstance().getRenderables(renderables, pool));
         }
         context.getControllerRenderables(renderables, pool);
         context.getBaseStationRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            context.getEyes().get(side).getCoordinateFrameInstance().getRenderables(renderables, pool);
         }
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
