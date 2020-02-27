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
   public ValkyrieCollisionMeshDefinitionDataHolder(ValkyrieJointMap jointMap, ValkyriePhysicalProperties physicalProperties)
   {
      RigidBodyTransform ankleOffset = new RigidBodyTransform();
      ankleOffset.appendTranslation((physicalProperties.getFootForward() - physicalProperties.getFootLength() * 0.5), 0.0,
                                    -physicalProperties.getAnkleHeight() * 0.5);

      CollisionMeshDefinitionData rightFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.RIGHT,
                                                                                                                           LegJointName.ANKLE_ROLL),
                                                                                                  physicalProperties.getFootLength(),
                                                                                                  physicalProperties.getFootWidth(),
                                                                                                  physicalProperties.getAnkleHeight());

      rightFootCollisionMeshData.setTransformToParentJoint(ankleOffset);
      addCollisionMeshDefinitionData(rightFootCollisionMeshData);

      CollisionMeshDefinitionData leftFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.LEFT,
                                                                                                                          LegJointName.ANKLE_ROLL),
                                                                                                 physicalProperties.getFootLength(),
                                                                                                 physicalProperties.getFootWidth(),
                                                                                                 physicalProperties.getAnkleHeight());

      leftFootCollisionMeshData.setTransformToParentJoint(ankleOffset);
      addCollisionMeshDefinitionData(leftFootCollisionMeshData);
   }
}
