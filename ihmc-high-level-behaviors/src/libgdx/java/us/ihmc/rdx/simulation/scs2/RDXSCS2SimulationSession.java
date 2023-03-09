package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsAsyncDebugger;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;

import java.util.Set;

public class RDXSCS2SimulationSession extends RDXSCS2Session
{
   private PhysicsEngine physicsEngine;
   private RDXBulletPhysicsAsyncDebugger bulletPhysicsDebugger;

   /**
    * Bring your own session.
    * This supports being called multiple times to switch between sessions.
    */
   public void startSession(SimulationSession simulationSession)
   {
      super.startSession(simulationSession);

      physicsEngine = simulationSession.getPhysicsEngine();
      if (physicsEngine instanceof BulletPhysicsEngine)
      {
         BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) physicsEngine;
         bulletPhysicsDebugger = new RDXBulletPhysicsAsyncDebugger(bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld());
      }

      simulationSession.addAfterPhysicsCallback(time ->
      {
         if (physicsEngine instanceof BulletPhysicsEngine)
            bulletPhysicsDebugger.drawBulletDebugDrawings();

         // This technically belongs in RDXSCS2Session, but there's no after tick callback for a regular session
         if (pauseAtEndOfBuffer.get() && yoManager.getCurrentIndex() == yoManager.getBufferSize() - 2)
         {
            session.setSessionMode(SessionMode.PAUSE);
         }
      });
   }

   @Override
   public void update()
   {
      super.update();

      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.update();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
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
