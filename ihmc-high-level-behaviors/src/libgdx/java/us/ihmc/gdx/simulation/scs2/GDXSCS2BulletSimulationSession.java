package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.bullet.GDXBulletPhysicsAsyncDebugger;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.tools.UnitConversions;

import java.util.Set;

public class GDXSCS2BulletSimulationSession extends GDXSCS2Session
{
   private GDXBulletPhysicsAsyncDebugger bulletPhysicsDebugger;
   private PhysicsEngine physicsEngine;

   public GDXSCS2BulletSimulationSession()
   {
      this(new SimulationSession(BulletPhysicsEngine::new));
   }

   public GDXSCS2BulletSimulationSession(SimulationSession simulationSession)
   {
      super(simulationSession);

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
      return getSimulationSession().addRobot(robotDefinition);
   }

   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      getSimulationSession().addTerrainObject(terrainObjectDefinition);
   }

   @Override
   public void update()
   {
      super.update();

      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.update();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<GDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(GDXSceneLevel.VIRTUAL))
      {
         if (physicsEngine instanceof BulletPhysicsEngine)
            bulletPhysicsDebugger.getVirtualRenderables(renderables, pool);
      }
   }

   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPartOne();

      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.renderImGuiWidgets();

      super.renderImGuiWidgetsPartTwo();
   }

   public SimulationSession getSimulationSession()
   {
      return (SimulationSession) session;
   }
}
