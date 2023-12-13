package us.ihmc.atlas.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.collisionShapes.CollisionBox;
import us.ihmc.perception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.perception.depthData.collisionShapes.CollisionShape;
import us.ihmc.perception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasCollisionBoxProvider implements CollisionBoxProvider
{
   private final Function<String, List<CollisionShape>> mappingFunction = s -> new ArrayList<>();
   private final HashMap<String, List<CollisionShape>> collisionMeshes = new HashMap<>();

   protected final float extent = 0.04f;

   public AtlasCollisionBoxProvider(RobotDefinition definitionWithCollisions, AtlasJointMap jointMap)
   {
      for (RigidBodyDefinition rigidBodyDefinition : definitionWithCollisions.getAllRigidBodies())
      {
         JointDefinition parentJoint = rigidBodyDefinition.getParentJoint();
         String jointName = parentJoint == null ? "root" : parentJoint.getName();

         for (CollisionShapeDefinition collisionShapeDefinition : rigidBodyDefinition.getCollisionShapeDefinitions())
         {
            RigidBodyTransform pose = new RigidBodyTransform(collisionShapeDefinition.getOriginPose());
            GeometryDefinition geometryDefinition = collisionShapeDefinition.getGeometryDefinition();

            if (geometryDefinition instanceof Box3DDefinition)
            {
               float bx = (float) (((Box3DDefinition) geometryDefinition).getSizeX() / 2.0) + extent;
               float by = (float) (((Box3DDefinition) geometryDefinition).getSizeY() / 2.0) + extent;
               float bz = (float) (((Box3DDefinition) geometryDefinition).getSizeZ() / 2.0) + extent;
               addCollisionShape(jointName, new CollisionBox(pose, bx, by, bz));
            }
            else if (geometryDefinition instanceof Cylinder3DDefinition)
            {
               float length = (float) (((Cylinder3DDefinition) geometryDefinition).getLength()) + 2.0f * extent;
               float radius = (float) (((Cylinder3DDefinition) geometryDefinition).getRadius()) + 2.0f * extent;
               addCollisionShape(jointName, new CollisionCylinder(pose, radius, length));
            }
            else if (geometryDefinition instanceof Sphere3DDefinition)
            {
               float radius = (float) (((Sphere3DDefinition) geometryDefinition).getRadius()) * 2.0f * extent;
               addCollisionShape(jointName, new CollisionSphere(pose, radius));
            }
            else
            {
               LogTools.debug("Cannot create collision box for " + rigidBodyDefinition);
               continue;
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         // Add mittens around the hands. This is about the size of the robotiq hands
         String joint = jointMap.getJointBeforeHandName(robotSide);
         RigidBodyTransform mittenPose = new RigidBodyTransform();
         mittenPose.getTranslation().set(0, robotSide.negateIfRightSide(0.27), -0.005);
         CollisionSphere hand = new CollisionSphere(mittenPose, 0.12 + extent);
         addCollisionShape(joint, hand);

         // Add extra collision sphere around the knee, to avoid hitting points there on the real robot
         String kneeJoint = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         RigidBodyTransform kneePose = new RigidBodyTransform();
         kneePose.setRotationRollAndZeroTranslation(Math.PI / 2.0);
         kneePose.getTranslation().set(0.05, 0, 0.03);
         CollisionCylinder knee = new CollisionCylinder(kneePose, 0.11, 0.15 + 2.0 * extent);
         addCollisionShape(kneeJoint, knee);

         // Add boxes for the stupid hose

         String elbowJoint = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
         RigidBodyTransform elbowBoxTransform = new RigidBodyTransform();
         elbowBoxTransform.getTranslation().set(0, robotSide.negateIfLeftSide(0.1), -0.2);
         CollisionBox stupidHoseBox = new CollisionBox(elbowBoxTransform, 0.15, 0.3, 0.15);
         addCollisionShape(elbowJoint, stupidHoseBox);
      }

      // Add wide backpack for hose

      String backRollJoint = jointMap.getSpineJointName(SpineJointName.SPINE_ROLL);
      RigidBodyTransform backPackTransform = new RigidBodyTransform();
      backPackTransform.getTranslation().set(-0.2, 0, 0.2);
      CollisionBox backPackHoseBox = new CollisionBox(backPackTransform, 0.2, 0.5, 0.3);
      addCollisionShape(backRollJoint, backPackHoseBox);

      RigidBodyTransform haloTransform = new RigidBodyTransform();
      haloTransform.getTranslation().set(0, 0, 1);
      CollisionCylinder halo = new CollisionCylinder(haloTransform, 0.35, 1.0);
      addCollisionShape(backRollJoint, halo);

   }

   public void addCollisionShape(String jointName, CollisionShape mesh)
   {
      collisionMeshes.computeIfAbsent(jointName, mappingFunction).add(mesh);
   }

   @Override
   public List<CollisionShape> getCollisionMesh(String jointName)
   {
      return collisionMeshes.get(jointName);
   }
}
