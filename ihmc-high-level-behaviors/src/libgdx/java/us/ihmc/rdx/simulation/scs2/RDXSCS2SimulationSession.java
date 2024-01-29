package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsAsyncDebugger;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletRobot;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;

import java.util.ArrayList;
import java.util.Set;

public class RDXSCS2SimulationSession extends RDXSCS2Session
{
   private final RDX3DPanel primary3DPanel;
   private PhysicsEngine physicsEngine;
   private RDXBulletPhysicsAsyncDebugger bulletPhysicsDebugger;
   private final ArrayList<RDXSCS2BulletRobotMover> robotMovers = new ArrayList<>();

   public RDXSCS2SimulationSession(RDXBaseUI baseUI)
   {
      super(baseUI);

      primary3DPanel = baseUI.getPrimary3DPanel();
   }

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

      robotMovers.clear();
      if (physicsEngine instanceof BulletPhysicsEngine bulletPhysicsEngine)
      {
         for (BulletRobot bulletRobot : bulletPhysicsEngine.getBulletRobots())
         {
            RDXSCS2BulletRobotMover robotMover = new RDXSCS2BulletRobotMover(primary3DPanel, bulletRobot);
            robotMovers.add(robotMover);
            simulationSession.addBeforePhysicsCallback(robotMover::beforePhysics);
            getAdditionalImGuiWidgets().add(robotMover::renderMoveJointCheckbox);
         }
      }

      for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
      {
         onSessionStartedRunnable.run();
      }
   }

   @Override
   public void update()
   {
      super.update();

      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.update();

      for (RDXSCS2BulletRobotMover robotMover : robotMovers)
         robotMover.update();
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

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPartOne();

      if (physicsEngine instanceof BulletPhysicsEngine)
         bulletPhysicsDebugger.renderImGuiWidgets();

      super.renderImGuiWidgetsPartTwo();
   }

   @Override
   public SimulationSession getSession()
   {
      return (SimulationSession) session;
   }

   public RDXBulletPhysicsAsyncDebugger getBulletPhysicsDebugger()
   {
      return bulletPhysicsDebugger;
   }
}
