package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.DurationCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;

public class GDXSCS2LogSession
{
   private final LogSession logSession;
   private final ImBoolean runAtRealtimeRate = new ImBoolean(true);
   private final ImDouble playbackRealtimeRate = new ImDouble(1.0);
   private final ImGuiPanel controlPanel = new ImGuiPanel("SCS 2 Log Session", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt bufferIndex = new ImInt();
   private final ImInt dtHz = new ImInt(-1);
   private final ImInt bufferRecordTickPeriod = new ImInt(1);
   private final ImFloat bufferDuration = new ImFloat(5.0f);
   private final ImBoolean pauseAtEndOfBuffer = new ImBoolean(true);
   private final ArrayList<Pair<ImBoolean, String>> showRobotPairs = new ArrayList<>();
   private final HashMap<String, ImBoolean> showRobotMap = new HashMap<>();
   private final ImBoolean showTerrain = new ImBoolean(true);
   private final ImBoolean showVirtualRenderables = new ImBoolean(true);
   private final ImBoolean showCollisionMeshes = new ImBoolean(false);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final GDXYoManager yoManager = new GDXYoManager();
   private final ArrayList<GDXSimulatedRobot> robots = new ArrayList<>();
   private final ArrayList<GDXSimulatedTerrainObject> terrainObjects = new ArrayList<>();
   private final SCS2YoImPlotManager plotManager = new SCS2YoImPlotManager();
   private boolean sessionStartedHandled = false;
   private final RenderableProvider getRealRenderables = this::getRealRenderables;
   private final RenderableProvider getVirtualRenderables = this::getVirtualRenderables;
   private ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();

   private final DurationCalculator simulationDurationCalculator = new DurationCalculator();

   public GDXSCS2LogSession(LogSession logSession)
   {
      this.logSession = logSession;
      dtHz.set((int) UnitConversions.secondsToHertz(logSession.getSessionDTSeconds()));

   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      create(baseUI, controlPanel);
   }

   public void create(GDXImGuiBasedUI baseUI, ImGuiPanel panel)
   {
      yoManager.startSession(logSession); // TODO: Add to controls?


      for (RobotDefinition robotDefinition : logSession.getRobotDefinitions())
      {
         GDXSimulatedRobot robot = new GDXSimulatedRobot(robotDefinition);
         robots.add(robot);
         robot.create(yoManager);
      }

      for (TerrainObjectDefinition terrainObjectDefinition : logSession.getTerrainObjectDefinitions())
      {
         GDXSimulatedTerrainObject simulatedTerrainObject = new GDXSimulatedTerrainObject(terrainObjectDefinition);
         terrainObjects.add(simulatedTerrainObject);
         simulatedTerrainObject.create();
      }

      logSession.getRootRegistry().addChild(yoRegistry);

      bufferRecordTickPeriod.set(logSession.getBufferRecordTickPeriod());
      changeBufferDuration(bufferDuration.get());
      updateDTFromSession();
      logSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      logSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());

      for (RobotDefinition robotDefinition : logSession.getRobotDefinitions())
      {
         ImBoolean imBoolean = new ImBoolean(true);
         showRobotPairs.add(ImmutablePair.of(imBoolean, robotDefinition.getName()));
         showRobotMap.put(robotDefinition.getName(), imBoolean);
      }

      logSession.startSessionThread(); // TODO: Need start/stop controls?

      baseUI.get3DSceneManager().addRenderableProvider(getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      baseUI.get3DSceneManager().addRenderableProvider(getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      plotManager.create(baseUI.getPerspectiveManager(), yoManager, panel);
   }

   public void update()
   {
      yoManager.update();

      if (!sessionStartedHandled && logSession.hasSessionStarted())
      {
         sessionStartedHandled = true;
         LogTools.info("Session started.");
         plotManager.initializeLinkedVariables();
         for (Runnable onSessionStartedRunnable : onSessionStartedRunnables)
         {
            onSessionStartedRunnable.run();
         }
      }

      if (logSession.getActiveMode() != SessionMode.RUNNING)
      {
         simulationDurationCalculator.pause();
      }

      for (GDXSimulatedRobot robot : robots)
      {
         robot.update();
      }
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXSimulatedRobot robot : robots)
      {
         if (showRobotMap.get(robot.getRobotDefinition().getName()).get())
         {
            robot.getRealRenderables(renderables, pool);
         }
      }
      if (showTerrain.get())
      {
         for (GDXSimulatedTerrainObject terrainObject : terrainObjects)
         {
            terrainObject.getRealRenderables(renderables, pool);
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showCollisionMeshes.get())
      {
         for (GDXSimulatedRobot robot : robots)
         {
            robot.getCollisionMeshRenderables(renderables, pool);
         }
         for (GDXSimulatedTerrainObject terrainObject : terrainObjects)
         {
            terrainObject.getCollisionRenderables(renderables, pool);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushItemWidth(110);
      if (ImGui.inputInt("Bullet simulation dt (Hz)", dtHz))
      {
         changeDT();
      }
      else
      {
         updateDTFromSession();
      }
      ImGui.popItemWidth();
      if (ImGui.radioButton("Run", logSession.getActiveMode() == SessionMode.RUNNING))
      {
         logSession.submitBufferIndexRequest(yoManager.getOutPoint());
         logSession.setSessionMode(SessionMode.RUNNING);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Pause", logSession.getActiveMode() == SessionMode.PAUSE))
      {
         logSession.setSessionMode(SessionMode.PAUSE);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Playback", logSession.getActiveMode() == SessionMode.PLAYBACK))
      {
         logSession.setSessionMode(SessionMode.PLAYBACK);
      }
      if (ImGui.button(labels.get("Run 1 tick")))
      {
         logSession.runTick();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Go to Out Point")))
      {
         logSession.submitBufferIndexRequest(yoManager.getOutPoint());
      }
      ImGui.sameLine();
      ImGui.text("Out point: " + yoManager.getOutPoint());
      if (ImGui.button(labels.get("Crop to In Out")))
      {
         CropBufferRequest cropBufferRequest = new CropBufferRequest(yoManager.getInPoint(), yoManager.getOutPoint());
         logSession.submitCropBufferRequest(cropBufferRequest);
      }
      if (ImGui.inputInt(labels.get("Buffer record tick period"), bufferRecordTickPeriod))
      {
         setBufferRecordTickPeriod(bufferRecordTickPeriod.get());
      }
      else
      {
         bufferRecordTickPeriod.set(logSession.getBufferRecordTickPeriod());
      }
      if (ImGui.inputFloat(labels.get("Buffer duration (s)"), bufferDuration))
      {
         changeBufferDuration(bufferDuration.get());
      }
      if (ImGui.sliderInt(labels.get("Buffer"), bufferIndex.getData(), 0, yoManager.getBufferSize()))
      {
         logSession.submitBufferIndexRequest(bufferIndex.get());
      }
      else
      {
         bufferIndex.set(yoManager.getCurrentIndex());
      }
      ImGui.checkbox(labels.get("Pause and end of buffer"), pauseAtEndOfBuffer);
      if (ImGui.checkbox("Run at real-time rate", runAtRealtimeRate))
      {
         logSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());
      }
      ImGui.pushItemWidth(100.0f);
      if (ImGui.inputDouble("Playback real-time rate", playbackRealtimeRate))
      {
         logSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      }
      ImGui.popItemWidth();
      for (Pair<ImBoolean, String> showRobotPair : showRobotPairs)
      {
         ImGui.checkbox(labels.get("Show " + showRobotPair.getRight()), showRobotPair.getLeft());
      }
      ImGui.checkbox(labels.get("Show terrain"), showTerrain);
      ImGui.checkbox(labels.get("Show virtual renderables"), showVirtualRenderables);
      ImGui.checkbox(labels.get("Show collision meshes"), showCollisionMeshes);
      plotManager.renderImGuiWidgets();
   }

   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      this.bufferRecordTickPeriod.set(bufferRecordTickPeriod);
      logSession.setBufferRecordTickPeriod(bufferRecordTickPeriod);
      changeBufferDuration(bufferDuration.get());
   }

   private void updateDTFromSession()
   {
      dtHz.set((int) UnitConversions.secondsToHertz(logSession.getSessionDTSeconds()));
   }

   public void changeBufferDuration(double bufferDuration)
   {
      this.bufferDuration.set((float) bufferDuration);
      int bufferSizeRequest = (int) (bufferDuration / UnitConversions.hertzToSeconds(dtHz.get()) / bufferRecordTickPeriod.get());
      logSession.submitBufferSizeRequest(bufferSizeRequest);
   }

   public void setPauseAtEndOfBuffer(boolean pauseAtEndOfBuffer)
   {
      this.pauseAtEndOfBuffer.set(pauseAtEndOfBuffer);
   }

   private void changeDT()
   {
      logSession.setSessionDTSeconds(UnitConversions.hertzToSeconds(dtHz.get()));
   }

   public void setDT(double dt)
   {
      dtHz.set((int) UnitConversions.secondsToHertz(dt));
      changeDT();
   }

   public void destroy(GDXImGuiBasedUI baseUI)
   {
      baseUI.get3DSceneManager().getSceneBasics().removeRenderableProvider(getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      baseUI.get3DSceneManager().getSceneBasics().removeRenderableProvider(getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      plotManager.destroy();

      showRobotMap.clear();
      showRobotPairs.clear();
   }

   public LogSession getSession()
   {
      return logSession;
   }

   public ImGuiPanel getControlPanel()
   {
      return controlPanel;
   }

   public GDXYoManager getYoManager()
   {
      return yoManager;
   }

   public HashMap<String, ImBoolean> getShowRobotMap()
   {
      return showRobotMap;
   }

   public ImBoolean getShowTerrain()
   {
      return showTerrain;
   }

   public ImBoolean getShowVirtualRenderables()
   {
      return showVirtualRenderables;
   }

   public ArrayList<Runnable> getOnSessionStartedRunnables()
   {
      return onSessionStartedRunnables;
   }
}
