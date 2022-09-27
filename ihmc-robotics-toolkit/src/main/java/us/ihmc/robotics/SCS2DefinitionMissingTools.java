package us.ihmc.robotics;

import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

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
}
