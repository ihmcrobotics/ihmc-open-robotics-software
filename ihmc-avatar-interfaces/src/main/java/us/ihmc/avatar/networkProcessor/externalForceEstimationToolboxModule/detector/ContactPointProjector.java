package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ContactPointProjector
{
   private final List<RigidBodyBasics> collidableRigidBodies = new ArrayList<>();
   private final Map<RigidBodyBasics, List<Collidable>> collidableMap = new HashMap<>();

   public ContactPointProjector(List<Collidable> allCollidables)
   {
      allCollidables.forEach(collidable -> collidableMap.computeIfAbsent(collidable.getRigidBody(), r -> new ArrayList<>()).add(collidable));
      collidableRigidBodies.addAll(collidableMap.keySet());
   }

   public boolean isPointInside(FramePoint3DBasics queryPoint, RigidBodyBasics rigidBody)
   {
      if (!collidableMap.containsKey(rigidBody))
      {
         throw new RuntimeException(rigidBody.getName() + " does not have any collidables");
      }

      List<Collidable> collidablesToCheck = collidableMap.get(rigidBody);

      for (int i = 0; i < collidablesToCheck.size(); i++)
      {
         FrameShape3DReadOnly collisionShape = collidablesToCheck.get(i).getShape();
         queryPoint.changeFrame(collisionShape.getReferenceFrame());

         if (collidablesToCheck.get(i).getShape().isPointInside(queryPoint))
         {
            return true;
         }
      }

      return false;
   }

   public void computeProjection(FramePoint3DBasics pointToProject, FramePoint3D contactPointPosition, FrameVector3D surfaceNormal, RigidBodyBasics rigidBody)
   {
      if (!collidableMap.containsKey(rigidBody))
      {
         throw new RuntimeException(rigidBody.getName() + " does not have any collidables");
      }

      List<Collidable> collidablesToCheck = collidableMap.get(rigidBody);

      double minimumProjectionDistance = Double.MAX_VALUE;
      int minimumDistanceIndex = -1;

      for (int i = 0; i < collidablesToCheck.size(); i++)
      {
         FrameShape3DReadOnly collisionShape = collidablesToCheck.get(i).getShape();
         pointToProject.changeFrame(collisionShape.getReferenceFrame());
         collisionShape.evaluatePoint3DCollision(pointToProject, contactPointPosition, surfaceNormal);

         boolean isInsideOthers = false;
         for (int j = i + 1; j < collidablesToCheck.size(); j++)
         {
            FrameShape3DReadOnly otherCollisionShape = collidablesToCheck.get(j).getShape();
            pointToProject.changeFrame(otherCollisionShape.getReferenceFrame());

            if (otherCollisionShape.isPointInside(contactPointPosition))
            {
               isInsideOthers = true;
               break;
            }
         }

         if (isInsideOthers)
         {
            continue;
         }

         pointToProject.changeFrame(contactPointPosition.getReferenceFrame());
         double distance = contactPointPosition.distance(pointToProject);
         if (distance < minimumProjectionDistance)
         {
            minimumProjectionDistance = distance;
            minimumDistanceIndex = i;
         }
      }

      FrameShape3DReadOnly shapeToProjectTo = collidablesToCheck.get(minimumDistanceIndex).getShape();
      pointToProject.changeFrame(shapeToProjectTo.getReferenceFrame());
      shapeToProjectTo.evaluatePoint3DCollision(pointToProject, contactPointPosition, surfaceNormal);
   }

   public List<RigidBodyBasics> getCollidableRigidBodies()
   {
      return collidableRigidBodies;
   }
}
