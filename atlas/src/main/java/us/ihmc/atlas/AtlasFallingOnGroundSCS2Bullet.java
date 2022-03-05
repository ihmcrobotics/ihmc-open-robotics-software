package us.ihmc.atlas;

import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.factory.TerrainObjectDefinitionTools;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

import java.util.Set;

public class AtlasFallingOnGroundSCS2Bullet
{
   public AtlasFallingOnGroundSCS2Bullet()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      double initialYaw = 0.3;
      double groundHeight = 0.1;
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
//      robotInitialSetup.initializeRobotDefinition(robotDefinition);

//      RigidBodyTransform transformToWorld = new RigidBodyTransform();
////      transformToWorld.getTranslation().setZ(2.0);
//      robotDefinition.getRootJointDefinitions().get(0).setTransformToParent(transformToWorld);
//      SixDoFJointDefinition sixDoFJointDefinition = (SixDoFJointDefinition) robotDefinition.getRootJointDefinitions().get(0);
//      SixDoFJointState initialJointState = new SixDoFJointState();
//      initialJointState.setPosition(new Point3D(0.0, 0.0, 1.0));
//      sixDoFJointDefinition.setInitialJointState(initialJointState);

      CollidableHelper collidableHelper = new CollidableHelper();
      RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(collidableHelper, "robot", "terrain");
      if (collisionModel != null)
         RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                             robotDefinition);
//      robotInitialSetup.initializeRobotDefinition(robotDefinition);
      Set<String> lastSimulatedJoints = robotModel.getJointMap().getLastSimulatedJoints();
      lastSimulatedJoints.forEach(robotDefinition::addSubtreeJointsToIgnore);

//      Robot robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
//      robot.updateFrames();
//      simulationSession.addRobot(robot);
      simulationSession.addRobot(robotDefinition);

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
//      simulationSession.addTerrainObject(TerrainObjectDefinitionTools.toTerrainObjectDefinition(environment,
//                                                                                                new CollidableHelper(),
//                                                                                                "terrain",
//                                                                                                "robot"));

      double groundWidth = 10.0;
      double groundLength = 10.0;
      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(1.5);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));
      simulationSession.addTerrainObject(terrain);

      SessionVisualizerControls sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
      sessionVisualizerControls.setCameraFocusPosition(0.3, 0.0, 1.0);
      sessionVisualizerControls.setCameraPosition(7.0, 4.0, 3.0);
   }

   public static void main(String[] args)
   {
      new AtlasFallingOnGroundSCS2Bullet();
   }
}
