package us.ihmc.exampleSimulations.cart;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CylinderCollisionMeshDefinitionData;

public class CartRobotCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   private double massBody = 100.0;
   private double xLengthBody = 0.8;
   private double yLengthBody = 0.4;
   private double zLengthBody = 0.1;

   private double heightPoll = 0.6;
   private double radiusPoll = 0.02;
   private double widthFlag = 0.4;
   private double heightFlag = 0.2;
   private double thicknessFlag = 0.01;

   private double massWheel = 50;
   private double wheelPlacementRatio = 0.8; // ratio with length of x from center to the end in x direction
   private double radiusWheel = 0.08;
   private double lengthWheel = yLengthBody * 0.8;

   public CartRobotCollisionMeshDefinitionDataHolder()
   {
      RigidBodyTransform transformToBody = new RigidBodyTransform();
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("body", xLengthBody, yLengthBody, zLengthBody);
      bodyCollisionMeshData.setTransformToParentJoint(transformToBody);
      addCollisionMeshDefinitionData(bodyCollisionMeshData);

      RigidBodyTransform transformToFrontWheel = new RigidBodyTransform();
      transformToFrontWheel.appendRollRotation(Math.PI / 2.0);
      transformToFrontWheel.appendTranslation(0.0, 0.0, -lengthWheel / 2.0);
      CollisionMeshDefinitionData frontWheelCollisionMeshData = new CylinderCollisionMeshDefinitionData("frontwheel", radiusWheel, lengthWheel);
      frontWheelCollisionMeshData.setTransformToParentJoint(transformToFrontWheel);
      addCollisionMeshDefinitionData(frontWheelCollisionMeshData);

      RigidBodyTransform transformToRearWheel = new RigidBodyTransform();
      transformToRearWheel.appendRollRotation(Math.PI / 2.0);
      transformToRearWheel.appendTranslation(0.0, 0.0, -lengthWheel / 2.0);
      CollisionMeshDefinitionData rearWheelCollisionMeshData = new CylinderCollisionMeshDefinitionData("rearwheel", radiusWheel, lengthWheel);
      rearWheelCollisionMeshData.setTransformToParentJoint(transformToRearWheel);
      addCollisionMeshDefinitionData(rearWheelCollisionMeshData);
   }
}
