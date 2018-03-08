package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public static double footLength = 0.265;
   public static double footWidth = 0.135;
   public static double footHeight = 0.06;

   public AtlasCollisionMeshDefinitionDataHolder(AtlasJointMap jointMap, AtlasPhysicalProperties atlasPhysicalProperties)
   {
      footLength = atlasPhysicalProperties.getActualFootLength();
      footWidth = atlasPhysicalProperties.getActualFootWidth();
      footHeight = Math.abs(atlasPhysicalProperties.getSoleToAnkleFrameTransforms().get(RobotSide.RIGHT).getM23());

      RigidBodyTransform transformToAnkle = new RigidBodyTransform(atlasPhysicalProperties.getSoleToAnkleFrameTransforms().get(RobotSide.RIGHT));
      transformToAnkle.appendTranslation(0.0, 0, 0.5 * footHeight);

      CollisionMeshDefinitionData rightFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.RIGHT,
                                                                                                                           LegJointName.ANKLE_ROLL),
                                                                                                  footLength, footWidth, footHeight);
      rightFootCollisionMeshData.setTransformToParentJoint(transformToAnkle);

      addCollisionMeshDefinitionData(rightFootCollisionMeshData);

      CollisionMeshDefinitionData leftFootCollisionMeshData = new BoxCollisionMeshDefinitionData(jointMap.getLegJointName(RobotSide.LEFT,
                                                                                                                          LegJointName.ANKLE_ROLL),
                                                                                                 footLength, footWidth, footHeight);
      leftFootCollisionMeshData.setTransformToParentJoint(transformToAnkle);

      addCollisionMeshDefinitionData(leftFootCollisionMeshData);
   }
}
