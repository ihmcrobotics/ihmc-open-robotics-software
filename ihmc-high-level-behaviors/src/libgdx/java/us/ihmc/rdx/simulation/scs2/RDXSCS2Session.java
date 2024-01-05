package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXRenderableAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.session.remote.RemoteSession;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.simulation.SimulationDataSession;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.DoNothingPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.StatelessNotification;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;

public class RDXSCS2Session
{
   protected Session session;
   private String sessionInfo = "";
   private final ImBoolean runAtRealtimeRate = new ImBoolean(true);
   private final ImDouble playbackRealtimeRate = new ImDouble(1.0);
   private final RDXPanel controlPanel = new RDXPanel("SCS 2 Session", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt bufferIndex = new ImInt();
   protected final ImInt dtHz = new ImInt(-1);
   private final ImInt bufferRecordTickPeriod = new ImInt(1);
   private final ImFloat bufferDuration = new ImFloat(5.0f);
   private String bufferSizeFormatString;
   protected final ImBoolean pauseAtEndOfBuffer = new ImBoolean(true);
   private final ArrayList<Pair<ImBoolean, String>> showRobotPairs = new ArrayList<>();
   private final HashMap<String, ImBoolean> showRobotMap = new HashMap<>();
   private final ImBoolean showTerrain = new ImBoolean(true);
   private final ImBoolean showVirtualRenderables = new ImBoolean(true);
   private final ImBoolean showCollisionMeshes = new ImBoolean(false);
   protected final RDXYoManager yoManager = new RDXYoManager();
   private final ArrayList<RDXSCS2Robot> robots = new ArrayList<>();
   private final ArrayList<RDXSCS2TerrainObject> terrainObjects = new ArrayList<>();
   private final RDXSCS2YoImPlotManager plotManager = new RDXSCS2YoImPlotManager();
   private boolean sessionStartedHandled = false;
   private final RDXRenderableAdapter renderables = new RDXRenderableAdapter(this::getRenderables);
   private final ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();
   private final ArrayList<Runnable> additionalImGuiWidgets = new ArrayList<>();
   private boolean stoppingSession = false;
   private final StatelessNotification sessionStoppedNotification = new StatelessNotification();

   public RDXSCS2Session(RDXBaseUI baseUI)
   {
      this(baseUI, null);
   }

   public RDXSCS2Session(RDXBaseUI baseUI, RDXPanel plotManagerParentPanel)
   {
      baseUI.getImGuiPanelManager().addPanel(controlPanel);

      baseUI.getPrimaryScene().addRenderableAdapter(renderables);
      plotManager.create(baseUI.getLayoutManager(), plotManagerParentPanel == null ? controlPanel : plotManagerParentPanel);
   }

   /**
    * This supports being called multiple times to switch between sessions.
    */
   public void startSession(Session session)
   {
      sessionStartedHandled = false;

      this.session = session;

      sessionInfo = "";
      if (session instanceof SimulationSession)
      {
         sessionInfo += "Simulation session";
      }
      else if (session instanceof SimulationDataSession)
      {
         sessionInfo += "Simulation data session";
      }
      else if (session instanceof LogSession)
      {
         sessionInfo += "Log session";
      }
      else if (session instanceof RemoteSession)
      {
         sessionInfo += "Remote session";
      }
      else
      {
         sessionInfo += session.getClass().getSimpleName();
      }
      sessionInfo += ": " + session.getSessionName();
      if (session instanceof SimulationSession simulationSession)
      {
         sessionInfo += "\nPhysics engine: ";
         if (simulationSession.getPhysicsEngine() instanceof BulletPhysicsEngine)
         {
            sessionInfo += "Bullet";
         }
         else if (simulationSession.getPhysicsEngine() instanceof ContactPointBasedPhysicsEngine)
         {
            sessionInfo += "Contact point";
         }
         else if (simulationSession.getPhysicsEngine() instanceof DoNothingPhysicsEngine)
         {
            sessionInfo += "Do nothing";
         }
         else if (simulationSession.getPhysicsEngine() instanceof ImpulseBasedPhysicsEngine)
         {
            sessionInfo += "Impulse based";
         }
         else
         {
            sessionInfo += simulationSession.getPhysicsEngine().getClass().getSimpleName();
         }
      }

      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));

      yoManager.startSession(session); // TODO: Add to controls?

      robots.clear();
      for (RobotDefinition robotDefinition : session.getRobotDefinitions())
      {
         RDXSCS2Robot robot = new RDXSCS2Robot(robotDefinition);
         robots.add(robot);
         robot.create(yoManager);
      }

      terrainObjects.clear();
      int terrainObjectIndex = 0;
      for (TerrainObjectDefinition terrainObjectDefinition : session.getTerrainObjectDefinitions())
      {
         RDXSCS2TerrainObject simulatedTerrainObject = new RDXSCS2TerrainObject(terrainObjectDefinition);
         if (session instanceof SimulationSession simulationSession
          && simulationSession.getPhysicsEngine() instanceof BulletPhysicsEngine bulletPhysicsEngine)
         { // Support moving terrain objects when using Bullet physics
            simulatedTerrainObject.setBulletTerrainObject(bulletPhysicsEngine.getTerrainObjects().get(terrainObjectIndex));
         }
         terrainObjects.add(simulatedTerrainObject);
         simulatedTerrainObject.create();
         ++terrainObjectIndex;
      }

      bufferRecordTickPeriod.set(session.getBufferRecordTickPeriod());
      changeBufferDuration(bufferDuration.get());
      updateDTFromSession();
      session.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      session.submitRunAtRealTimeRate(runAtRealtimeRate.get());

      showRobotPairs.clear();
      showRobotMap.clear();
      for (RobotDefinition robotDefinition : session.getRobotDefinitions())
      {
         ImBoolean imBoolean = new ImBoolean(true);
         showRobotPairs.add(ImmutablePair.of(imBoolean, robotDefinition.getName()));
         showRobotMap.put(robotDefinition.getName(), imBoolean);
      }

      plotManager.setupForSession(yoManager);

      session.startSessionThread(); // TODO: Need start/stop controls?
   }

   public void update()
   {
      yoManager.update();
      plotManager.update();

      if (!sessionStartedHandled && session.hasSessionStarted())
      {
         sessionStartedHandled = true;
         LogTools.info("Session started.");
         plotManager.initializeLinkedVariables();
      }

      for (RDXSCS2Robot robot : robots)
      {
         robot.update();
      }
      for (RDXSCS2TerrainObject terrainObject : terrainObjects)
      {
         terrainObject.update();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.GROUND_TRUTH))
      {
         for (RDXSCS2Robot robot : robots)
         {
            ImBoolean showRobot = showRobotMap.get(robot.getRobotDefinition().getName());
            if (showRobot != null && showRobot.get()) // Sometimes it's not ready yet and can be null
            {
               robot.getRealRenderables(renderables, pool);
            }
         }
         if (showTerrain.get())
         {
            for (RDXSCS2TerrainObject terrainObject : terrainObjects)
            {
               terrainObject.getRealRenderables(renderables, pool);
            }
         }
      }
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (showCollisionMeshes.get())
         {
            for (RDXSCS2Robot robot : robots)
            {
               robot.getCollisionMeshRenderables(renderables, pool);
            }
            for (RDXSCS2TerrainObject terrainObject : terrainObjects)
            {
               terrainObject.getCollisionRenderables(renderables, pool);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPartOne();
      renderImGuiWidgetsPartTwo();
   }

   protected void renderImGuiWidgetsPartOne()
   {
      ImGui.text(sessionInfo);
      ImGui.pushItemWidth(110);
      if (ImGuiTools.volatileInputInt("DT (Hz)", dtHz))
      {
         changeDT();
      }
      else
      {
         updateDTFromSession();
      }
      ImGui.popItemWidth();
      ImGui.text("Session Time: " + session.getTime().getValue());
      if (ImGui.radioButton("Run", session.getActiveMode() == SessionMode.RUNNING))
      {
         session.submitBufferIndexRequest(yoManager.getOutPoint());
         session.setSessionMode(SessionMode.RUNNING);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Pause", session.getActiveMode() == SessionMode.PAUSE))
      {
         session.setSessionMode(SessionMode.PAUSE);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Playback", session.getActiveMode() == SessionMode.PLAYBACK))
      {
         session.setSessionMode(SessionMode.PLAYBACK);
      }
      if (ImGui.button(labels.get("Run 1 tick")))
      {
         session.runTick();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("<")))
      {
         session.submitDecrementBufferIndexRequest(1);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         session.submitIncrementBufferIndexRequest(1);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Go to Out Point")))
      {
         session.submitBufferIndexRequest(yoManager.getOutPoint());
      }
      ImGui.sameLine();
      ImGui.text("Out point: " + yoManager.getOutPoint());
      if (ImGui.button(labels.get("Crop to In Out")))
      {
         CropBufferRequest cropBufferRequest = new CropBufferRequest(yoManager.getInPoint(), yoManager.getOutPoint());
         session.submitCropBufferRequest(cropBufferRequest);
      }
      ImGui.pushItemWidth(80.0f);
      if (ImGuiTools.volatileInputInt(labels.get("Buffer record tick period"), bufferRecordTickPeriod))
      {
         setBufferRecordTickPeriod(bufferRecordTickPeriod.get());
      }
      else
      {
         bufferRecordTickPeriod.set(session.getBufferRecordTickPeriod());
      }

      if (ImGuiTools.volatileInputFloat(labels.get("Buffer duration (s)"), bufferDuration))
      {
         changeBufferDuration(bufferDuration.get());
      }
      ImGui.popItemWidth();
      if (ImGui.sliderInt(labels.get("Buffer"), bufferIndex.getData(), 0, yoManager.getBufferSize(), bufferSizeFormatString))
      {
         session.submitBufferIndexRequest(bufferIndex.get());
      }
      else
      {
         bufferIndex.set(yoManager.getCurrentIndex());
      }
      ImGui.checkbox(labels.get("Pause and end of buffer"), pauseAtEndOfBuffer);
      if (ImGui.checkbox("Run at real-time rate", runAtRealtimeRate))
      {
         session.submitRunAtRealTimeRate(runAtRealtimeRate.get());
      }
      ImGui.pushItemWidth(100.0f);
      if (ImGuiTools.volatileInputDouble("Playback real-time rate", playbackRealtimeRate))
      {
         session.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      }
      ImGui.popItemWidth();
      for (Pair<ImBoolean, String> showRobotPair : showRobotPairs)
      {
         ImGui.checkbox(labels.get("Show " + showRobotPair.getRight()), showRobotPair.getLeft());
      }
      ImGui.checkbox(labels.get("Show terrain"), showTerrain);
      ImGui.checkbox(labels.get("Show virtual renderables"), showVirtualRenderables);
      ImGui.checkbox(labels.get("Show collision meshes"), showCollisionMeshes);
   }

   protected void renderImGuiWidgetsPartTwo()
   {
      plotManager.renderImGuiWidgets();

      for (Runnable additionalImGuiWidget : additionalImGuiWidgets)
      {
         additionalImGuiWidget.run();
      }
   }

   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      this.bufferRecordTickPeriod.set(bufferRecordTickPeriod);
      session.setBufferRecordTickPeriod(bufferRecordTickPeriod);
      changeBufferDuration(bufferDuration.get());
   }

   private void updateDTFromSession()
   {
      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));
   }

   public void changeBufferDuration(double bufferDuration)
   {
      this.bufferDuration.set((float) bufferDuration);
      int bufferSizeRequest = (int) (bufferDuration / UnitConversions.hertzToSeconds(dtHz.get()) / bufferRecordTickPeriod.get());
      bufferSizeFormatString = "%i / " + bufferSizeRequest;
      session.submitBufferSizeRequest(bufferSizeRequest);
   }

   public void setPauseAtEndOfBuffer(boolean pauseAtEndOfBuffer)
   {
      this.pauseAtEndOfBuffer.set(pauseAtEndOfBuffer);
   }

   private void changeDT()
   {
      session.setSessionDTSeconds(UnitConversions.hertzToSeconds(dtHz.get()));
   }

   public void setDT(double dt)
   {
      dtHz.set((int) UnitConversions.secondsToHertz(dt));
      changeDT();
   }

   public void endSession()
   {
      stoppingSession = true;
      ThreadTools.startAsDaemon(() ->
      {
         session.stopSessionThread();
         stoppingSession = false;
         sessionStoppedNotification.notifyOtherThread();
      }, getClass().getSimpleName() + "Destroy");
      robots.clear();
      terrainObjects.clear();
      showRobotPairs.clear();
      showRobotMap.clear();
   }

   public void waitForSessionToBeStopped()
   {
      if (stoppingSession)
         sessionStoppedNotification.blockingWait();
   }

   public void destroy(RDXBaseUI baseUI)
   {
      baseUI.getPrimaryScene().removeRenderableAdapter(renderables);

      plotManager.destroy();

      showRobotMap.clear();
      showRobotPairs.clear();
   }

   public ArrayList<RDXSCS2Robot> getRobots()
   {
      return robots;
   }

   public Session getSession()
   {
      return session;
   }

   public RDXYoManager getYoManager()
   {
      return yoManager;
   }

   public RDXSCS2YoImPlotManager getPlotManager()
   {
      return plotManager;
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

   public double getSimDT()
   {
      return UnitConversions.hertzToSeconds(dtHz.get());
   }

   public int getBufferRecordTickPeriod()
   {
      return bufferRecordTickPeriod.get();
   }

   public ArrayList<Runnable> getAdditionalImGuiWidgets()
   {
      return additionalImGuiWidgets;
   }

   public boolean isSessionThreadRunning()
   {
      return session.hasSessionStarted() && !session.isSessionShutdown();
   }
}
