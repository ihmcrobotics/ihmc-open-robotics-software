package us.ihmc.atlas;

import us.ihmc.avatar.scs2.SCS2BulletSimulationTools;
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
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.simulationToolkit.RobotDefinitionTools;

import java.util.Set;

public class AtlasFallingOnGroundSCS2Bullet
{
   public AtlasFallingOnGroundSCS2Bullet()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      robotModel.setRobotDefinitionMutator(robotDefinition ->
      {
      });

      boolean removeCollisions = false;
      MaterialDefinition materialDefinition = null; // default material
      RobotDefinition robotDefinition = robotModel.createRobotDefinition(materialDefinition, removeCollisions);

      if (removeCollisions)
      {
         RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
         if (collisionModel != null)
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
      }
      else
      {
         setCollisionMasks(robotDefinition);
      }

      Set<String> lastSimulatedJoints = robotModel.getJointMap().getLastSimulatedJoints();
      lastSimulatedJoints.forEach(robotDefinition::addSubtreeJointsToIgnore);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);
      simulationSession.addRobot(robotDefinition);

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
      //simulationSession.addRobot(boxRobotDefinition);

      SessionVisualizer sessionVisualizer = SCS2BulletSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);

      sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.3, 0.0, 1.0);
      sessionVisualizer.getSessionVisualizerControls().setCameraPosition(7.0, 4.0, 3.0);
      sessionVisualizer.getToolkit().getSession().runTick();
   }

   private void setCollisionMasks(RobotDefinition robotDefinition)
   {
      String defaultFilter = "DefaultFilter";
      String staticFilter = "StaticFilter";
      String kinematicFilter = "KinematicFilter";
      String debrisFilter = "DebrisFilter";
      String sensorTrigger = "SensorTrigger";
      String characterFilter = "CharacterFilter";
      String bodyName = "Body"; 
      String pelvis = "Pelvis";
      String rightLeg = "RightLeg";
      String leftLeg = "LeftLeg";
      String rightArm = "RightArm";
      String leftArm = "LeftArm";
      long bulletCollisionGroup;
      long bulletCollideMask;

      CollidableHelper helper = new CollidableHelper();

      // Set default Bullet collision groups/masks
      bulletCollisionGroup = helper.getCollisionMask(defaultFilter);
      bulletCollisionGroup = helper.getCollisionMask(staticFilter);
      bulletCollisionGroup = helper.getCollisionMask(kinematicFilter);
      bulletCollisionGroup = helper.getCollisionMask(debrisFilter);
      bulletCollisionGroup = helper.getCollisionMask(sensorTrigger);
      bulletCollisionGroup = helper.getCollisionMask(characterFilter);
      
      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (CollisionShapeDefinition shapeDefinition : rigidBodyDefinition.getCollisionShapeDefinitions())
         {
            if (shapeDefinition.getName().contains("pelvis")
                  || shapeDefinition.getName().contains("uglut")
                  || shapeDefinition.getName().contains("lglut")
                  )
            {
               bulletCollisionGroup = helper.getCollisionMask(pelvis);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, leftArm, rightArm);

            }
            else if (shapeDefinition.getName().contains("utorso")
                  || shapeDefinition.getName().contains("hokuyo") 
                  || shapeDefinition.getName().contains("head"))
            {
               bulletCollisionGroup = helper.getCollisionMask(bodyName);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, rightLeg, leftLeg, rightArm, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uleg")
                  || shapeDefinition.getName().contains("l_lleg")
                  || shapeDefinition.getName().contains("l_talus")
                  || shapeDefinition.getName().contains("l_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, rightLeg, leftArm);

            }
            else if (shapeDefinition.getName().contains("r_uleg")
                  || shapeDefinition.getName().contains("r_lleg")
                  || shapeDefinition.getName().contains("r_talus")
                  || shapeDefinition.getName().contains("r_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, leftLeg, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uarm")
                  || shapeDefinition.getName().contains("l_clav")
                  || shapeDefinition.getName().contains("l_scap")
                  || shapeDefinition.getName().contains("l_larm")
                  || shapeDefinition.getName().contains("l_ufarm")
                  || shapeDefinition.getName().contains("l_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, pelvis, leftLeg, rightLeg, rightArm);
            }
            else if (shapeDefinition.getName().contains("r_uarm")
                  || shapeDefinition.getName().contains("r_clav")
                  || shapeDefinition.getName().contains("r_scap")
                  || shapeDefinition.getName().contains("r_larm")
                  || shapeDefinition.getName().contains("r_ufarm")
                  || shapeDefinition.getName().contains("r_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, pelvis, leftLeg, rightLeg, leftArm);
            }
            else
            {
               bulletCollisionGroup = 1;
               bulletCollideMask = 1 + 2;
            }

            //bullet has it the opposite names as CollidableHelper e.g. a collisionMask is a collisionGroup in bullet
            shapeDefinition.setCollisionMask(bulletCollisionGroup);
            shapeDefinition.setCollisionGroup(bulletCollideMask);
         }
      }
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName,
                                                     Tuple3DReadOnly size,
                                                     double mass,
                                                     double radiusOfGyrationPercent,
                                                     ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size, mass, radiusOfGyrationPercent, null, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName,
                                                     Tuple3DReadOnly size,
                                                     double mass,
                                                     double radiusOfGyrationPercent,
                                                     Vector3DReadOnly offsetFromParent,
                                                     ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, offsetFromParent, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName,
                                                     double sizeX,
                                                     double sizeY,
                                                     double sizeZ,
                                                     double mass,
                                                     double radiusOfGyrationPercent,
                                                     Vector3DReadOnly offsetFromParentJoint,
                                                     ColorDefinition color)
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
