package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;

import java.util.ArrayList;

public class GDXSCS2PhysicsSimulator
{
   private final SimulationSession simulationSession;
   private final ImBoolean runAtRealtimeRate;
   private final ImDouble playbackRealtimeRate;
   private final ImGuiPanel controlPanel = new ImGuiPanel("Physics Simulator", this::renderImGuiWidgets);
   private final GDXYoManager yoManager = new GDXYoManager();
   private final ArrayList<GDXSimulatedRobot> robots = new ArrayList<>();
   private final ArrayList<GDXSimulatedTerrainObject> terrainObjects = new ArrayList<>();

   public GDXSCS2PhysicsSimulator()
   {
      simulationSession = new SimulationSession();
      runAtRealtimeRate = new ImBoolean(simulationSession.getRunAtRealTimeRate());
      playbackRealtimeRate = new ImDouble(simulationSession.getPlaybackRealTimeRate());
   }

   public void addRobot(RobotDefinition robotDefinition)
   {
      GDXSimulatedRobot robot = new GDXSimulatedRobot(robotDefinition);
      robots.add(robot);
      simulationSession.addRobot(robotDefinition);
   }

   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      simulationSession.addTerrainObject(terrainObjectDefinition);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      yoManager.startSession(simulationSession); // TODO: Add to controls?

      for (GDXSimulatedRobot robot : robots)
      {
         robot.create(yoManager);
      }

      for (TerrainObjectDefinition terrainObjectDefinition : simulationSession.getTerrainObjectDefinitions())
      {
         GDXSimulatedTerrainObject simulatedTerrainObject = new GDXSimulatedTerrainObject(terrainObjectDefinition);
         terrainObjects.add(simulatedTerrainObject);
         simulatedTerrainObject.create();
      }

      simulationSession.startSessionThread(); // TODO: Need start/stop controls?

      baseUI.get3DSceneManager().addRenderableProvider(this::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void update()
   {
      yoManager.update();
      for (GDXSimulatedRobot robot : robots)
      {
         robot.update();
      }
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

   private void renderImGuiWidgets()
   {
      if (ImGui.radioButton("Simulate", simulationSession.getActiveMode() == SessionMode.RUNNING))
      {
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
