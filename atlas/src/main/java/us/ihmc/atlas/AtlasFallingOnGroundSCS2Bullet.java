package us.ihmc.atlas;

import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.factory.TerrainObjectDefinitionTools;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.*;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

import java.util.List;
import java.util.Set;

public class AtlasFallingOnGroundSCS2Bullet
{
   public AtlasFallingOnGroundSCS2Bullet()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      robotModel.setRobotDefinitionMutator(robotDefinition -> {});

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

      RigidBodyDefinition pelvisRigidBody = robotDefinition.getRigidBodyDefinition("pelvis");
      pelvisRigidBody.setMass(1.0);
//      pelvisRigidBody.getInertiaPose().setToZero();
      List<CollisionShapeDefinition> pelvisCollisions = pelvisRigidBody.getCollisionShapeDefinitions();
      pelvisCollisions.clear();
      CollisionShapeDefinition pelvisBoxCollisionDefinition = new CollisionShapeDefinition();
      pelvisBoxCollisionDefinition.setGeometryDefinition(new Box3DDefinition(0.3, 0.3, 0.3));
      pelvisBoxCollisionDefinition.setOriginPose(new RigidBodyTransform());
      pelvisCollisions.add(pelvisBoxCollisionDefinition);
      pelvisRigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(pelvisRigidBody.getMass(),
                                                                                     0.8 * 0.3,
                                                                                     0.8 * 0.3,
                                                                                     0.8 * 0.3));
      robotDefinition.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(0.1, 0.1, 0.1), new Point3D(1.5, 0.0, 0.0)));

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

      RobotDefinition boxRobotDefinition = new RobotDefinition("atlas2");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("elevator2");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("pelvis2");
      rootJointDefinition.setInitialJointState(new SixDoFJointState(new YawPitchRoll(0.1, 0.1, 0.1), new Point3D(0.6, 0.0, 0.0)));
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      Vector3D boxSize1 = new Vector3D(0.3, 0.3, 0.3);
      double boxMass1 = 1.0;
      double radiusOfGyrationPercent = 0.8;
      ColorDefinition boxApp1 = ColorDefinitions.LightSeaGreen();
      RigidBodyDefinition rigidBody1 = newBoxRigidBody("pelvis2", boxSize1, boxMass1, radiusOfGyrationPercent, boxApp1);
      rigidBody1.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize1)));
      rootJointDefinition.setSuccessor(rigidBody1);
      boxRobotDefinition.setRootBodyDefinition(rootBodyDefinition);
//      simulationSession.addRobot(boxRobotDefinition);

      SessionVisualizerControls sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
      sessionVisualizerControls.setCameraFocusPosition(0.3, 0.0, 1.0);
      sessionVisualizerControls.setCameraPosition(7.0, 4.0, 3.0);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent,
                                                     ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size, mass, radiusOfGyrationPercent, null, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent,
                                                     Vector3DReadOnly offsetFromParent, ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, offsetFromParent, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                                     double radiusOfGyrationPercent, Vector3DReadOnly offsetFromParentJoint, ColorDefinition color)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(rigidBodyName);
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * sizeX,
                                                                                     radiusOfGyrationPercent * sizeY,
                                                                                     radiusOfGyrationPercent * sizeZ));
      if (offsetFromParentJoint != null)
         rigidBody.setCenterOfMassOffset(offsetFromParentJoint);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      if (offsetFromParentJoint != null)
         factory.appendTranslation(offsetFromParentJoint);
      factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(color));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }

   public static void main(String[] args)
   {
      new AtlasFallingOnGroundSCS2Bullet();
   }
}
