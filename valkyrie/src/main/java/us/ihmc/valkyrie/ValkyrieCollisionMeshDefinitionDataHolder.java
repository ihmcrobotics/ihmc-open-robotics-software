package us.ihmc.valkyrie;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;

public class ValkyrieCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public ValkyrieCollisionMeshDefinitionDataHolder(ValkyrieJointMap jointMap)
   {
      RigidBodyTransform ankleOffset = new RigidBodyTransform();
      ankleOffset.appendTranslation((ValkyriePhysicalProperties.footForward - ValkyriePhysicalProperties.footLength * 0.5), 0.0,
                                    -ValkyriePhysicalProperties.ankleHeight * 0.5);

      CollisionMeshDefinitionData rightFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.RIGHT,
                                                                                                                           LegJointName.ANKLE_ROLL),
                                                                                                  ValkyriePhysicalProperties.footLength,
                                                                                                  ValkyriePhysicalProperties.footWidth,
                                                                                                  ValkyriePhysicalProperties.ankleHeight);

      rightFootCollisionMeshData.setTransformToParentJoint(ankleOffset);
      addCollisionMeshDefinitionData(rightFootCollisionMeshData);

      CollisionMeshDefinitionData leftFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.LEFT,
                                                                                                                          LegJointName.ANKLE_ROLL),
                                                                                                 ValkyriePhysicalProperties.footLength,
                                                                                                 ValkyriePhysicalProperties.footWidth,
                                                                                                 ValkyriePhysicalProperties.ankleHeight);

      leftFootCollisionMeshData.setTransformToParentJoint(ankleOffset);
      addCollisionMeshDefinitionData(leftFootCollisionMeshData);
   }
}
