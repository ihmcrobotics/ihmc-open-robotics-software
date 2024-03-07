package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiTableFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.time.FrequencyCalculator;

public class RDXVRManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RDXVRContext context = new RDXVRContext();
   private Notification contextCreatedNotification;
   private boolean contextInitialized = false;
   private boolean initializing = false;
   private boolean skipHeadset = false;
   private final Object syncObject = new Object();
   private final ImBoolean showScenePoseGizmo = new ImBoolean(false);
   private RDXPose3DGizmo scenePoseGizmo;
   private final ImBoolean vrEnabled = new ImBoolean(false);
   private final Notification posesReady = new Notification();
   private volatile boolean waitingOnPoses = false;
   private ImGuiPlot vrFPSPlot = new ImGuiPlot(labels.get("VR FPS Hz"), 1000, 180, 50);
   private FrequencyCalculator vrFPSCalculator = new FrequencyCalculator();
   private ImGuiPlot waitGetPosesPlot = new ImGuiPlot(labels.get("Wait Get Poses Hz"), 1000, 180, 50);
   private ImGuiPlot waitGetToRenderDelayPlot = new ImGuiPlot(labels.get("WaitGetToRender Delay"), 1000, 180, 50);
   private final Stopwatch waitGetToRenderStopwatch = new Stopwatch();
   private volatile double waitGetToRenderDuration = Double.NaN;
   private FrequencyCalculator waitGetPosesFrequencyCalculator = new FrequencyCalculator();
   private ImGuiPlot pollEventsPlot = new ImGuiPlot(labels.get("Poll Events Hz"), 1000, 180, 50);
   private FrequencyCalculator pollEventsFrequencyCalculator = new FrequencyCalculator();
   private ImGuiPlot contextInitializedPlot = new ImGuiPlot(labels.get("contextInitialized"), 1000, 180, 50);
   private ImGuiPlot initSystemCountPlot = new ImGuiPlot(labels.get("initSystemCount"), 1000, 180, 50);
   private volatile int initSystemCount = 0;
   private ImGuiPlot setupEyesCountPlot = new ImGuiPlot(labels.get("setupEyesCount"), 1000, 180, 50);
   private volatile int setupEyesCount = 0;
   private final Notification waitOnPosesNotification = new Notification();
   private volatile boolean waitGetPosesThreadRunning = false;
   private final RDXVRTeleporter teleporter = new RDXVRTeleporter();

   public void create()
   {
      teleporter.create(context);
   }

   /**
    * Doing poll and render close together makes VR performance the best it can be
    * and reduce dizziness.
    *
    * TODO: This thread has to be a multiple of the parent (240?)
    * TODO: If the rest of the app is too slow, can we run this one faster?
    */
   public void pollEventsAndRender(RDXBaseUI baseUI, RDX3DScene scene)
   {
      boolean posesReady = pollEvents(baseUI);
      if (posesReady && isVRReady())
      {
         skipHeadset = true;
         vrFPSCalculator.ping();
         waitGetToRenderDuration = waitGetToRenderStopwatch.totalElapsed();
         synchronized (syncObject)
         {
            context.renderEyes(scene);
         }
         skipHeadset = false;
      }
   }

   private boolean pollEvents(RDXBaseUI baseUI)
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
               synchronized (syncObject)
               {
                  initSystemCount++;
                  context.initSystem();
               }
               contextCreatedNotification.set();
            });
         }
         if (contextCreatedNotification != null && contextCreatedNotification.poll())
         {
            initializing = false;

            synchronized (syncObject)
            {
               setupEyesCount++;
               context.setupEyes();
            }

            if (!Boolean.parseBoolean(System.getProperty("rdx.free.spin")))
            {
               baseUI.setForegroundFPSLimit(350); // TODO: Do something better with this
            }
            baseUI.setVsync(false); // important to disable vsync for VR
            scenePoseGizmo = new RDXPose3DGizmo(context.getTeleportFrameIHMCZUp(), context.getTeleportIHMCZUpToIHMCZUpWorld());
            scenePoseGizmo.create(baseUI.getPrimary3DPanel());
            contextInitialized = true;
            waitGetPosesThreadRunning = true;
            ThreadTools.startAsDaemon(this::waitOnPoses, getClass().getSimpleName() + "WaitOnPosesThread");
         }

         if (contextInitialized)
         {
            // Waiting for the poses on a thread allows for the rest of the application to keep
            // cranking faster than VR can do stuff. This also makes the performance graph in Steam
            // show the correct value and the OpenVR stack work much better.
            // TODO: This whole thing might have major issues because
            // there's a delay waiting for the next time this method is called
            posesReadyThisFrame = posesReady.poll();

            if (!posesReadyThisFrame && !waitingOnPoses)
            {
               waitingOnPoses = true;
               waitOnPosesNotification.set();
            }
            else
            {
               waitingOnPoses = false;
            }

            if (posesReadyThisFrame)
            {
               pollEventsFrequencyCalculator.ping();
               synchronized (syncObject)
               {
                  context.pollEvents(); // FIXME: Potential bug is that the poses get updated in the above thread while they're being used in here
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

      return posesReadyThisFrame;
   }

   private void waitOnPoses()
   {
      while (waitGetPosesThreadRunning)
      {
         waitOnPosesNotification.blockingPoll();

         if (waitGetPosesThreadRunning)
         {
            waitGetPosesFrequencyCalculator.ping();
            synchronized (syncObject)
            {
               context.waitGetPoses();
            }
            waitGetToRenderStopwatch.reset();
            posesReady.set();
         }
      }
   }

   public void renderMenuBar()
   {
      ImGui.setNextWindowSize(350.0f, 250.0f);
      if (ImGui.beginMenu(labels.get("VR")))
      {
         ImGui.text("Connected headset: " + (isVRReady() ? context.getHeadset().getModelName() : "None"));
         if (ImGui.beginTable(labels.get("vrTable"), 2, ImGuiTableFlags.None))
         {
            // First row (libgdx log level)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            renderEnableCheckbox();
            ImGui.tableSetColumnIndex(1);

            // Second row (theme)
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.checkbox(labels.get("Controllers only"), new ImBoolean());
            ImGui.tableSetColumnIndex(1);

            ImGui.endTable();
         }
         if (ImGui.collapsingHeader(labels.get("Debug")))
         {
            renderDebugPlots();
         }
         ImGui.endMenu();
      }
   }

   public void renderEnableCheckbox()
   {
      if (ImGui.checkbox(labels.get("VR Enabled"), vrEnabled))
      {
         if (vrEnabled.get())
         {
            RDXBaseUI.pushNotification("Enabling VR...");

            ThreadTools.startAThread(() ->
            {
               ThreadTools.sleep(5000);

               if (isVRReady())
               {
                  RDXBaseUI.pushNotification("VR enabled.");
               }
               else
               {
                  RDXBaseUI.pushNotification("Unable to enable VR.");
                  vrEnabled.set(false);
               }
            }, getClass().getName() + "VREnableMonitor");
         }
         else
         {
            RDXBaseUI.pushNotification("VR disabled");
         }
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("Start SteamVR and turn on controllers before enabling.");
      }
   }

   public void renderDebugPlots()
   {
      ImGui.checkbox(labels.get("Show scene pose gizmo"), showScenePoseGizmo);
      contextInitializedPlot.render(contextInitialized ? 1.0 : 0.0);
      initSystemCountPlot.render(initSystemCount);
      setupEyesCountPlot.render(setupEyesCount);
      waitGetPosesPlot.render(waitGetPosesFrequencyCalculator.getFrequency());
      pollEventsPlot.render(pollEventsFrequencyCalculator.getFrequency());
      vrFPSPlot.render(vrFPSCalculator.getFrequency());
      waitGetToRenderDelayPlot.render(waitGetToRenderDuration);
   }

   public void renderImGuiTunerWidgets()
   {
      for (RobotSide side : RobotSide.values)
      {
         RDXVRController controller = context.getController(side);
         ImGui.text(side.getPascalCaseName() + " controller selection point:");
         controller.renderImGuiTunerWidgets();
      }
   }

   public void dispose()
   {
      waitGetPosesThreadRunning = false;
      waitOnPosesNotification.set();
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

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (isVRReady() && showScenePoseGizmo.get())
      {
         scenePoseGizmo.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (isVRReady() && showScenePoseGizmo.get())
      {
         scenePoseGizmo.process3DViewInput(input);
         context.teleport(teleportIHMCZUpToIHMCZUpWorldConsumer ->
         {
            teleportIHMCZUpToIHMCZUpWorldConsumer.set(scenePoseGizmo.getTransformToParent());
         });
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (vrEnabled.get() && contextInitialized)
      {
         teleporter.getRenderables(renderables, pool);
         if (!skipHeadset)
         {
            context.getHeadsetRenderable(renderables, pool);
         }
         context.getControllerRenderables(renderables, pool);
         context.getTrackerRenderables(renderables, pool);
         if (showScenePoseGizmo.get())
            scenePoseGizmo.getRenderables(renderables, pool);
      }
   }

   public RDXVRContext getContext()
   {
      return context;
   }

   public RDXVRTeleporter getTeleporter()
   {
      return teleporter;
   }
}
