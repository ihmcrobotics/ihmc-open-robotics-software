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
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
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
   private final ImFloat bufferLength = new ImFloat(5.0f);
   private final ImBoolean pauseAtEndOfBuffer = new ImBoolean(true);
   private final ImBoolean showCollisionMeshes = new ImBoolean(false);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final DurationCalculator simulationDurationCalculator = new DurationCalculator();
   private final YoDouble simulationRealtimeRate = new YoDouble("simulationRealtimeRate", yoRegistry);
   private final GDXYoManager yoManager = new GDXYoManager();
   private final ArrayList<GDXSimulatedRobot> robots = new ArrayList<>();
   private final ArrayList<GDXSimulatedTerrainObject> terrainObjects = new ArrayList<>();
   private final GDXBulletPhysicsAsyncDebugger bulletPhysicsDebugger;
   private final SCS2YoImPlotManager plotManager = new SCS2YoImPlotManager();
   private boolean sessionStartedHandled = false;
   private final RenderableProvider getRealRenderables = this::getRealRenderables;
   private final RenderableProvider getVirtualRenderables = this::getVirtualRenderables;

   public GDXSCS2SimulationSession()
   {
      this(new SimulationSession(BulletPhysicsEngine::new));
   }

   public GDXSCS2SimulationSession(SimulationSession simulationSession)
   {
      this.simulationSession = simulationSession;

      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
      bulletPhysicsDebugger = new GDXBulletPhysicsAsyncDebugger(bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld());

      simulationSession.addAfterPhysicsCallback(time ->
      {
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

      changeBufferLength();
      changeDT();
      simulationSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      simulationSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());

      simulationSession.startSessionThread(); // TODO: Need start/stop controls?

      baseUI.get3DSceneManager().addRenderableProvider(getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      baseUI.get3DSceneManager().addRenderableProvider(getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      plotManager.create(baseUI.getPerspectiveManager(), yoManager, controlPanel);
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
      bulletPhysicsDebugger.getVirtualRenderables(renderables, pool);
   }

   private void renderImGuiWidgets()
   {
      ImGui.pushItemWidth(110);
      if (ImGui.inputInt("Bullet simulation dt (Hz)", dtHz))
      {
         changeDT();
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
      if (ImGui.inputFloat(labels.get("Buffer length (s)"), bufferLength))
      {
         changeBufferLength();
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
      bulletPhysicsDebugger.renderImGuiWidgets();
      plotManager.renderImGuiWidgets();
   }

   private void changeBufferLength()
   {
      int bufferSizeRequest = (int) (bufferLength.get() / UnitConversions.hertzToSeconds(dtHz.get()));
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
