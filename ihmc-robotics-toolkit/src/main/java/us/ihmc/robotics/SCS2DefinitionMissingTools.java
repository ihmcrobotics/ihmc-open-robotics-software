package us.ihmc.robotics;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.util.List;
import java.util.function.Consumer;

public class SCS2DefinitionMissingTools
{
   public static void forEachRigidBodyDefinitionIncludingFourBars(RigidBodyDefinition start, Consumer<RigidBodyDefinition> rigidBodyConsumer)
   {
      RobotDefinition.forEachRigidBodyDefinition(start, body ->
      {
         rigidBodyConsumer.accept(body);
         for (JointDefinition childrenJoint : body.getChildrenJoints())
         {
            if (childrenJoint instanceof CrossFourBarJointDefinition fourBarJointDefinition)
            {
               rigidBodyConsumer.accept(fourBarJointDefinition.getBodyBC());
               rigidBodyConsumer.accept(fourBarJointDefinition.getBodyDA());
            }
         }
      });
   }

   public static void addCollisionVisualsToRobot(RobotDefinition robotDefinition, MaterialDefinition material)
   {
      robotDefinition.forEachRigidBodyDefinition(rigidBody -> addCollisionVisualsToRigidBodyDefinition(rigidBody, material));
   }

   public static void addCollisionVisualsToRigidBodyDefinition(RigidBodyDefinition rigidBodyDefinition, MaterialDefinition material)
   {
      if (rigidBodyDefinition == null)
         return;
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();
      if (collisionShapeDefinitions == null)
         return;

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         rigidBodyDefinition.addVisualDefinition(new VisualDefinition(collisionShapeDefinition.getOriginPose(),
                                                                      collisionShapeDefinition.getGeometryDefinition(),
                                                                      material));
      }
   }
}
