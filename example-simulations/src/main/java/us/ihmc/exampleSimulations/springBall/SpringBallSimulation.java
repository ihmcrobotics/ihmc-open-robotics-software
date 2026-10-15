package us.ihmc.exampleSimulations.springBall;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class SpringBallSimulation
{
   public SpringBallSimulation()
   {
      SpringBallRobotDefinition springBall = new SpringBallRobotDefinition();

      // Same stiffness/damping values as the original SCS1 LinearGroundContactModel(springBall, 400.0, 10.0, 80.0, 100.0, ...).
      ContactPointBasedContactParameters contactParameters = new ContactPointBasedContactParameters();
      contactParameters.setKxy(400.0);
      contactParameters.setBxy(10.0);
      contactParameters.setKz(80.0);
      contactParameters.setBz(100.0);

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactory.newContactPointBasedPhysicsEngineFactory(contactParameters));
      simulationSession.addRobot(springBall);
      simulationSession.addTerrainObject(flatGround());

      SimulationSessionControls simulationSessionControls = simulationSession.getSimulationSessionControls();
      simulationSessionControls.setDT(0.002);
      simulationSessionControls.setBufferRecordTickPeriod(5);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   private static TerrainObjectDefinition flatGround()
   {
      RigidBodyTransform originPose = new RigidBodyTransform();
      originPose.appendTranslation(0.0, 0.0, -0.25);

      GeometryDefinition groundGeometry = new Box3DDefinition(10000.0, 10000.0, 0.50);

      TerrainObjectDefinition terrain = new TerrainObjectDefinition();
      terrain.addVisualDefinition(new VisualDefinition(originPose, groundGeometry, new MaterialDefinition(ColorDefinitions.DarkGray())));
      terrain.addCollisionShapeDefinition(new CollisionShapeDefinition(originPose, groundGeometry));
      return terrain;
   }

   public static void main(String[] args)
   {
      new SpringBallSimulation();
   }
}
