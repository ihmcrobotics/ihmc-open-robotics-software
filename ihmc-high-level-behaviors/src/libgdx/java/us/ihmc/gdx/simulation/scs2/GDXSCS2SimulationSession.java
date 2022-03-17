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
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.time.DurationCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class GDXSCS2SimulationSession
{
   private final SimulationSession simulationSession;
   private final ImBoolean runAtRealtimeRate = new ImBoolean(true);
   private final ImDouble playbackRealtimeRate = new ImDouble(1.0);
   private final ImGuiPanel controlPanel = new ImGuiPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt bufferIndex = new ImInt();
   private final ImInt dtHz = new ImInt(240);
   private final ImInt bufferRecordTickPeriod = new ImInt(1);
   private final ImFloat bufferDuration = new ImFloat(5.0f);
   private final ImBoolean pauseAtEndOfBuffer = new ImBoolean(true);
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

   public GDXSCS2SimulationSession()
   {
      this(new SimulationSession(BulletPhysicsEngine::new));
   }

   public GDXSCS2SimulationSession(SimulationSession simulationSession)
   {
      this.simulationSession = simulationSession;

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

   public void addRobot(RobotDefinition robotDefinition)
   {
      simulationSession.addRobot(robotDefinition);
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
      changeBufferDuration();
      changeDT();
      simulationSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      simulationSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());

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
         robot.getRealRenderables(renderables, pool);
      }
      for (GDXSimulatedTerrainObject terrainObject : terrainObjects)
      {
         terrainObject.getRealRenderables(renderables, pool);
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
         dtHz.set((int) UnitConversions.secondsToHertz(simulationSession.getSessionDTSeconds()));
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
         simulationSession.submitBufferRecordTickPeriod(bufferRecordTickPeriod.get());
         changeBufferDuration();
      }
      else
      {
         bufferRecordTickPeriod.set(simulationSession.getBufferRecordTickPeriod());
      }
      if (ImGui.inputFloat(labels.get("Buffer duration (s)"), bufferDuration))
      {
         changeBufferDuration();
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
      ImGui.checkbox(labels.get("Show collision meshes"), showCollisionMeshes);
      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.renderImGuiWidgets();
      plotManager.renderImGuiWidgets();
   }

   private void changeBufferDuration()
   {
      int bufferSizeRequest = (int) (bufferDuration.get() / UnitConversions.hertzToSeconds(dtHz.get()) / bufferRecordTickPeriod.get());
      simulationSession.submitBufferSizeRequest(bufferSizeRequest);
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
}
