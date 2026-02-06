package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

public class RDXVRManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RDXVRContext context = new RDXVRContext();
   private boolean contextInitialized = false;
   private boolean skipHeadset = false;
   private final ImBoolean showScenePoseGizmo = new ImBoolean(false);
   private RDXPose3DGizmo scenePoseGizmo;
   private final ImBoolean vrEnabled = new ImBoolean(false);
   private final ImGuiPlot vrFPSPlot = new ImGuiPlot(labels.get("VR FPS Hz"), 1000, 180, 50);
   private final FrequencyCalculator vrFPSCalculator = new FrequencyCalculator();
   private final ImGuiPlot waitGetToRenderDelayPlot = new ImGuiPlot(labels.get("WaitGetToRender Delay"), 1000, 180, 50);
   private final Stopwatch waitGetToRenderStopwatch = new Stopwatch();
   private volatile double waitGetToRenderDuration = Double.NaN;
   private final ImGuiPlot pollEventsPlot = new ImGuiPlot(labels.get("Poll Events Hz"), 1000, 180, 50);
   private final FrequencyCalculator pollEventsFrequencyCalculator = new FrequencyCalculator();
   private final ImGuiPlot contextInitializedPlot = new ImGuiPlot(labels.get("contextInitialized"), 1000, 180, 50);
   private final RDXVRTeleporter teleporter = new RDXVRTeleporter();
   private final List<RDXVRTrackerRoleManager> trackerRoleManagers = new ArrayList<>();

   public void create()
   {
      teleporter.create(context);

      for (RDXVRTracker tracker : context.getTrackers().values())
         trackerRoleManagers.add(new RDXVRTrackerRoleManager(context, tracker));
   }

   public void pollEventsAndRender(RDXBaseUI baseUI, RDX3DScene scene)
   {
      pollEvents(baseUI);
      if (isVRReady())
      {
         skipHeadset = true;
         vrFPSCalculator.ping();
         waitGetToRenderDuration = waitGetToRenderStopwatch.totalElapsed();
         context.renderEyes(scene);
         skipHeadset = false;
      }
   }

   private void pollEvents(RDXBaseUI baseUI)
   {
      // Close VR if it was previously initialized and now was disabled
      if (!vrEnabled.get() && contextInitialized)
      {
         dispose();
      }
      else if (vrEnabled.get())
      {
         if (!contextInitialized)
         {
            context.initSystem(); // May block for a bit
            context.setupEyes();

            scenePoseGizmo = new RDXPose3DGizmo(context.getTeleportFrameIHMCZUp(), context.getTeleportIHMCZUpToIHMCZUpWorld());
            scenePoseGizmo.create(baseUI.getPrimary3DPanel());
            contextInitialized = true;
         }

         context.waitGetPoses();
         waitGetToRenderStopwatch.reset();

         // pollEventsFrequencyCalculator.ping();
         context.pollEvents();

         // A tracker has disconnected
         List<String> removedTrackersSerialNumbers = context.getRemovedTrackersSerialNumbers();
         Iterator<RDXVRTrackerRoleManager> trackerIterator = trackerRoleManagers.iterator();
         while (trackerIterator.hasNext())
         {
            RDXVRTrackerRoleManager tracker = trackerIterator.next();

            if (removedTrackersSerialNumbers.contains(tracker.getTrackerSerialNumber()))
            {
               trackerIterator.remove();

               LogTools.warn("Tracker {} removed", tracker.getTrackerSerialNumber());
            }
         }

         // A new tracker has been detected
         List<String> newTrackersSerialNumbers = context.getNewTrackersSerialNumbers();
         for (String newSerialNumber : newTrackersSerialNumbers)
         {
            trackerRoleManagers.add(new RDXVRTrackerRoleManager(context, context.getTrackers().get(newSerialNumber)));
         }

         // A reset of roles has been triggered from the UI
         if (context.getRolesResetNotification().poll())
         {
            for (var trackerRoleManager : trackerRoleManagers)
            {
               trackerRoleManager.reset();
            }
         }

         // A loading of preset roles has been triggered from the UI
         if (context.getLoadingRolesNotification().poll())
         {
            var trackerRoleMap = context.getTrackersRoleMap();
            for (var trackerRole : trackerRoleMap.entrySet())
            {
               for (var trackerRoleManager : trackerRoleManagers)
               {
                  // if serial numbers match
                  if (trackerRole.getValue().equals(trackerRoleManager.getTrackerSerialNumber()))
                  {
                     trackerRoleManager.setActive(trackerRole.getKey());
                  }
               }
            }
            LogTools.info("Loaded roles");
         }
      }
   }

   public void renderMenuBar()
   {
      ImGui.setNextWindowSize(350.0f, 250.0f);
      if (ImGui.beginMenu(labels.get("VR")))
      {
         ImGuiTools.separatorText("Controls");
         renderEnableCheckbox();

         ImGuiTools.separatorText("Status");
         if (isVRReady())
         {
            ImGui.text("Connected headset: " + context.getHeadset().getModelName());
            ImGui.text("Connected controllers: " + StringUtils.join(context.getControllers()
                                                                           .values()
                                                                           .stream()
                                                                           .filter(RDXVRTrackedDevice::isConnected)
                                                                           .collect(Collectors.toList()), ", "));
            ImGui.text("Connected trackers: " + StringUtils.join(context.getTrackers()
                                                                        .values()
                                                                        .stream()
                                                                        .filter(RDXVRTrackedDevice::isConnected)
                                                                        .collect(Collectors.toList()), ", "));
         }
         else
         {
            ImGui.text("VR not enabled");
         }

         if (ImGui.collapsingHeader(labels.get("Debug")))
            renderDebugPlots();

         ImGui.endMenu();
      }
   }

   public void renderEnableCheckbox()
   {
      if (ImGui.menuItem(labels.get("VR Enabled"), "", vrEnabled))
      {
         if (vrEnabled.get())
         {
            enableVR();
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
      if (contextInitialized)
      {
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
            context.getHeadset().getRenderables(renderables, pool);
         context.getControllerRenderables(renderables, pool);
         for (var trackerRoleManager : trackerRoleManagers)
         {
            if (!trackerRoleManager.isRoleAssigned())
               trackerRoleManager.getRedModelInstance().getRenderables(renderables, pool);
            else
               context.getTrackers().get(trackerRoleManager.getTrackerSerialNumber()).getRenderables(renderables, pool);
         }
         if (showScenePoseGizmo.get())
            scenePoseGizmo.getRenderables(renderables, pool);
      }
   }

   public void enableVR()
   {
      vrEnabled.set(true);
      RDXBaseUI.pushNotification("Enabling VR...");

      ThreadTools.startAThread(() ->
      {
          ThreadTools.sleep(5000);

          if (isVRReady())
          {
             RDXBaseUI.pushNotification("VR enabled");
          }
          else
          {
             RDXBaseUI.pushNotification("Unable to enable VR");
             vrEnabled.set(false);
          }
      }, getClass().getName() + "VREnableMonitor");
   }

   public RDXVRContext getContext()
   {
      return context;
   }

   public RDXVRTeleporter getTeleporter()
   {
      return teleporter;
   }

   public List<RDXVRTrackerRoleManager> getTrackerRoleManagers()
   {
      return trackerRoleManagers;
   }
}
