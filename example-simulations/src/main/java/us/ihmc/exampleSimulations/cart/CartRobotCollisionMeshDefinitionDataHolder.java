package us.ihmc.exampleSimulations.cart;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CylinderCollisionMeshDefinitionData;

public class CartRobotCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   private final double xLengthBody = 0.8;
   private final double yLengthBody = 0.4;
   private final double zLengthBody = 0.1;

   private final double radiusWheel = 0.08;
   private final double lengthWheel = yLengthBody * 0.8;

   public CartRobotCollisionMeshDefinitionDataHolder()
   {
      RigidBodyTransform transformToBody = new RigidBodyTransform();
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("body", xLengthBody, yLengthBody, zLengthBody);
      bodyCollisionMeshData.setTransformToParentJoint(transformToBody);
      addCollisionMeshDefinitionData(bodyCollisionMeshData);

      RigidBodyTransform transformToFrontWheel = new RigidBodyTransform();
      transformToFrontWheel.appendRollRotation(Math.PI / 2.0);
      transformToFrontWheel.appendTranslation(0.0, 0.0, -lengthWheel / 2.0);
      CollisionMeshDefinitionData frontWheelCollisionMeshData = new CylinderCollisionMeshDefinitionData("frontWheel", radiusWheel, lengthWheel);
      frontWheelCollisionMeshData.setTransformToParentJoint(transformToFrontWheel);
      addCollisionMeshDefinitionData(frontWheelCollisionMeshData);

      RigidBodyTransform transformToRearWheel = new RigidBodyTransform();
      transformToRearWheel.appendRollRotation(Math.PI / 2.0);
      transformToRearWheel.appendTranslation(0.0, 0.0, -lengthWheel / 2.0);
      CollisionMeshDefinitionData rearWheelCollisionMeshData = new CylinderCollisionMeshDefinitionData("rearWheel", radiusWheel, lengthWheel);
      rearWheelCollisionMeshData.setTransformToParentJoint(transformToRearWheel);
      addCollisionMeshDefinitionData(rearWheelCollisionMeshData);
   }
}
