package us.ihmc.avatar.scs2;

import javafx.application.Platform;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

public class SCS2BulletSimulationTools
{
   public static SessionVisualizer startSessionVisualizerWithDebugDrawing(SimulationSession simulationSession)
   {
      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
//      BulletDebugDrawingNode bulletDebugDrawingNode = new BulletDebugDrawingNode(bulletPhysicsEngine.getBulletMultiBodyDynamicsWorld());
//      simulationSession.getRootRegistry().addChild(bulletDebugDrawingNode.getYoRegistry());

      simulationSession.initializeBufferSize(24000);

      SessionVisualizer sessionVisualizer = SessionVisualizer.startSessionVisualizerExpert(simulationSession, null);

      Platform.runLater(() ->
      {
//         bulletDebugDrawingNode.initializeWithJavaFX();
//         sessionVisualizer.getToolkit().getEnvironmentManager().getRootNode().getChildren().add(bulletDebugDrawingNode);
      });

      return sessionVisualizer;
   }
}
