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
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.bullet.GDXBulletPhysicsAsyncDebugger;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.DurationCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.HashMap;

public class GDXSCS2SimulationSession
{
   private static final int MAX_SCS2_BUFFER_SIZE = 10000;
   private final SimulationSession simulationSession;
   private final ImBoolean runAtRealtimeRate = new ImBoolean(true);
   private final ImDouble playbackRealtimeRate = new ImDouble(1.0);
   private final ImGuiPanel controlPanel = new ImGuiPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
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
   private final DurationCalculator simulationDurationCalculator = new DurationCalculator();
   private final YoDouble simulationRealtimeRate = new YoDouble("simulationRealtimeRate", yoRegistry);
   private final GDXYoManager yoManager = new GDXYoManager();
   private final ArrayList<GDXSimulatedRobot> robots = new ArrayList<>();
   private final ArrayList<GDXSimulatedTerrainObject> terrainObjects = new ArrayList<>();
   private GDXBulletPhysicsAsyncDebugger bulletPhysicsDebugger;
   private final SCS2YoImPlotManager plotManager = new SCS2YoImPlotManager();
   private boolean sessionStartedHandled = false;
   private final RenderableProvider getRealRenderables = this::getRealRenderables;
   private final RenderableProvider getVirtualRenderables = this::getVirtualRenderables;
   private PhysicsEngine physicsEngine;
   private ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();

   public GDXSCS2SimulationSession()
   {
      this(new SimulationSession(BulletPhysicsEngine::new));
   }

   public GDXSCS2SimulationSession(SimulationSession simulationSession)
   {
      this.simulationSession = simulationSession;
      dtHz.set((int) UnitConversions.secondsToHertz(simulationSession.getSessionDTSeconds()));

      physicsEngine = simulationSession.getPhysicsEngine();
      if (physicsEngine instanceof BulletPhysicsEngine)
      {
         BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) physicsEngine;
         bulletPhysicsDebugger = new GDXBulletPhysicsAsyncDebugger(bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld());
      }

      simulationSession.addAfterPhysicsCallback(time ->
      {
         if (physicsEngine instanceof BulletPhysicsEngine)
            bulletPhysicsDebugger.drawBulletDebugDrawings();

         if (pauseAtEndOfBuffer.get() && yoManager.getCurrentIndex() == yoManager.getBufferSize() - 2)
         {
            simulationSession.setSessionMode(SessionMode.PAUSE);
         }

         simulationDurationCalculator.ping();
         simulationRealtimeRate.set(UnitConversions.hertzToSeconds(dtHz.get()) / simulationDurationCalculator.getDuration());
      });
   }

   public Robot addRobot(RobotDefinition robotDefinition)
   {
      return simulationSession.addRobot(robotDefinition);
   }

   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      simulationSession.addTerrainObject(terrainObjectDefinition);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      create(baseUI, controlPanel);
   }

   public void create(GDXImGuiBasedUI baseUI, ImGuiPanel panel)
   {
      yoManager.startSession(simulationSession); // TODO: Add to controls?

      for (RobotDefinition robotDefinition : simulationSession.getRobotDefinitions())
      {
         GDXSimulatedRobot robot = new GDXSimulatedRobot(robotDefinition);
         robots.add(robot);
         robot.create(yoManager);
      }

      for (TerrainObjectDefinition terrainObjectDefinition : simulationSession.getTerrainObjectDefinitions())
      {
         GDXSimulatedTerrainObject simulatedTerrainObject = new GDXSimulatedTerrainObject(terrainObjectDefinition);
         terrainObjects.add(simulatedTerrainObject);
         simulatedTerrainObject.create();
      }

      simulationSession.getRootRegistry().addChild(yoRegistry);

      bufferRecordTickPeriod.set(simulationSession.getBufferRecordTickPeriod());
      changeBufferDuration(bufferDuration.get());
      updateDTFromSession();
      simulationSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      simulationSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());

      for (RobotDefinition robotDefinition : simulationSession.getRobotDefinitions())
      {
         ImBoolean imBoolean = new ImBoolean(true);
         showRobotPairs.add(ImmutablePair.of(imBoolean, robotDefinition.getName()));
         showRobotMap.put(robotDefinition.getName(), imBoolean);
      }

      simulationSession.startSessionThread(); // TODO: Need start/stop controls?

      baseUI.get3DSceneManager().addRenderableProvider(getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      baseUI.get3DSceneManager().addRenderableProvider(getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      plotManager.create(baseUI.getPerspectiveManager(), yoManager, panel);
   }

   public void update()
   {
      yoManager.update();

      if (!sessionStartedHandled && simulationSession.hasSessionStarted())
      {
         sessionStartedHandled = true;
         LogTools.info("Session started.");
         plotManager.initializeLinkedVariables();
         for (Runnable onSessionStartedRunnable : onSessionStartedRunnables)
         {
            onSessionStartedRunnable.run();
         }
      }

      if (simulationSession.getActiveMode() != SessionMode.RUNNING)
      {
         simulationDurationCalculator.pause();
      }

      for (GDXSimulatedRobot robot : robots)
      {
         robot.update();
      }
      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.update();
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
      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.getVirtualRenderables(renderables, pool);
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
      if (ImGui.radioButton("Run", simulationSession.getActiveMode() == SessionMode.RUNNING))
      {
         simulationSession.submitBufferIndexRequest(yoManager.getOutPoint());
         simulationSession.setSessionMode(SessionMode.RUNNING);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Pause", simulationSession.getActiveMode() == SessionMode.PAUSE))
      {
         simulationSession.setSessionMode(SessionMode.PAUSE);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Playback", simulationSession.getActiveMode() == SessionMode.PLAYBACK))
      {
         simulationSession.setSessionMode(SessionMode.PLAYBACK);
      }
      if (ImGui.button(labels.get("Run 1 tick")))
      {
         simulationSession.runTick();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Go to Out Point")))
      {
         simulationSession.submitBufferIndexRequest(yoManager.getOutPoint());
      }
      ImGui.sameLine();
      ImGui.text("Out point: " + yoManager.getOutPoint());
      if (ImGui.button(labels.get("Crop to In Out")))
      {
         CropBufferRequest cropBufferRequest = new CropBufferRequest(yoManager.getInPoint(), yoManager.getOutPoint());
         simulationSession.submitCropBufferRequest(cropBufferRequest);
      }
      if (ImGui.inputInt(labels.get("Buffer record tick period"), bufferRecordTickPeriod))
      {
         setBufferRecordTickPeriod(bufferRecordTickPeriod.get());
      }
      else
      {
         bufferRecordTickPeriod.set(simulationSession.getBufferRecordTickPeriod());
      }
      if (ImGui.inputFloat(labels.get("Buffer duration (s)"), bufferDuration))
      {
         changeBufferDuration(bufferDuration.get());
      }
      if (ImGui.sliderInt(labels.get("Buffer"), bufferIndex.getData(), 0, yoManager.getBufferSize()))
      {
         simulationSession.submitBufferIndexRequest(bufferIndex.get());
      }
      else
      {
         bufferIndex.set(yoManager.getCurrentIndex());
      }
      ImGui.checkbox(labels.get("Pause and end of buffer"), pauseAtEndOfBuffer);
      if (ImGui.checkbox("Run at real-time rate", runAtRealtimeRate))
      {
         simulationSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());
      }
      ImGui.pushItemWidth(100.0f);
      if (ImGui.inputDouble("Playback real-time rate", playbackRealtimeRate))
      {
         simulationSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      }
      ImGui.popItemWidth();
      for (Pair<ImBoolean, String> showRobotPair : showRobotPairs)
      {
         ImGui.checkbox(labels.get("Show " + showRobotPair.getRight()), showRobotPair.getLeft());
      }
      ImGui.checkbox(labels.get("Show terrain"), showTerrain);
      ImGui.checkbox(labels.get("Show virtual renderables"), showVirtualRenderables);
      ImGui.checkbox(labels.get("Show collision meshes"), showCollisionMeshes);
      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.renderImGuiWidgets();
      plotManager.renderImGuiWidgets();
   }

   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      this.bufferRecordTickPeriod.set(bufferRecordTickPeriod);
      simulationSession.setBufferRecordTickPeriod(bufferRecordTickPeriod);
      changeBufferDuration(bufferDuration.get());
   }

   private void updateDTFromSession()
   {
      dtHz.set((int) UnitConversions.secondsToHertz(simulationSession.getSessionDTSeconds()));
   }

   public void changeBufferDuration(double bufferDuration)
   {
      this.bufferDuration.set((float) bufferDuration);
      int bufferSizeRequest = (int) (bufferDuration / UnitConversions.hertzToSeconds(dtHz.get()) / bufferRecordTickPeriod.get());
      if (bufferSizeRequest > MAX_SCS2_BUFFER_SIZE)
      {
         LogTools.warn("Trying to set buffer size to {}, clamping it to: {}", bufferSizeRequest, MAX_SCS2_BUFFER_SIZE);
         bufferSizeRequest = MAX_SCS2_BUFFER_SIZE;
      }
      
      simulationSession.submitBufferSizeRequest(bufferSizeRequest);
   }

   public void setPauseAtEndOfBuffer(boolean pauseAtEndOfBuffer)
   {
      this.pauseAtEndOfBuffer.set(pauseAtEndOfBuffer);
   }

   private void changeDT()
   {
      simulationSession.setSessionDTSeconds(UnitConversions.hertzToSeconds(dtHz.get()));
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

   public SimulationSession getSession()
   {
      return simulationSession;
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
