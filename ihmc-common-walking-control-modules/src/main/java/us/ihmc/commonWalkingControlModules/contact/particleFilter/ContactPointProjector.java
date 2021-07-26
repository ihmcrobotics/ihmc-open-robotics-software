package us.ihmc.commonWalkingControlModules.contact.particleFilter;

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
   private final RigidBodyBasics[] collidableRigidBodies;
   private final Map<RigidBodyBasics, List<Collidable>> collidableMap = new HashMap<>();

   public ContactPointProjector(List<Collidable> allCollidables)
   {
      allCollidables.forEach(collidable -> collidableMap.computeIfAbsent(collidable.getRigidBody(), r -> new ArrayList<>()).add(collidable));
      collidableRigidBodies = collidableMap.keySet().toArray(new RigidBodyBasics[0]);
   }

   public boolean isPointInsideAnyRigidBody(FramePoint3DBasics queryPoint)
   {
      return isPointInside(queryPoint, collidableRigidBodies);
   }

   public boolean isPointInside(FramePoint3DBasics queryPoint, RigidBodyBasics... rigidBodiesToCheck)
   {
      for (int i = 0; i < rigidBodiesToCheck.length; i++)
      {
         List<Collidable> collidablesToCheck = collidableMap.get(rigidBodiesToCheck[i]);

         for (int j = 0; j < collidablesToCheck.size(); j++)
         {
            FrameShape3DReadOnly collisionShape = collidablesToCheck.get(j).getShape();
            queryPoint.changeFrame(collisionShape.getReferenceFrame());

            if (collidablesToCheck.get(j).getShape().isPointInside(queryPoint))
            {
               return true;
            }
         }
      }

      return false;
   }

   public List<RigidBodyBasics> computeCollidingRigidBodies(FramePoint3DBasics queryPoint)
   {
      List<RigidBodyBasics> collidingRigidBodies = new ArrayList<>();

      outerLoop:
      for (int i = 0; i < collidableRigidBodies.length; i++)
      {
         List<Collidable> collidablesToCheck = collidableMap.get(collidableRigidBodies[i]);

         for (int j = 0; j < collidablesToCheck.size(); j++)
         {
            FrameShape3DReadOnly collisionShape = collidablesToCheck.get(j).getShape();
            queryPoint.changeFrame(collisionShape.getReferenceFrame());

            if (collidablesToCheck.get(j).getShape().isPointInside(queryPoint))
            {
               collidingRigidBodies.add(collidablesToCheck.get(j).getRigidBody());
               continue outerLoop;
            }
         }
      }

      return collidingRigidBodies;
   }

   public void projectToSpecificLink(FramePoint3DBasics pointToProject,
                                     FramePoint3D contactPointPosition,
                                     FrameVector3D surfaceNormal,
                                     RigidBodyBasics rigidBody)
   {
      projectPoint(pointToProject, contactPointPosition, surfaceNormal, rigidBody);
   }

   public RigidBodyBasics projectToClosestLink(FramePoint3DBasics pointToProject, FramePoint3D contactPointPosition, FrameVector3D surfaceNormal)
   {
      return projectPoint(pointToProject, contactPointPosition, surfaceNormal, collidableRigidBodies);
   }

   /**
    * Projects the given point to the surface of the nearest collidable.
    * Assumes the point isn't inside a collidable, use {@link #isPointInsideAnyRigidBody} to check
    */
   public RigidBodyBasics projectPoint(FramePoint3DBasics pointToProject,
                                       FramePoint3D contactPointPosition,
                                       FrameVector3D surfaceNormal,
                                       RigidBodyBasics... rigidBodiesToCheck)
   {
      double minimumProjectionDistance = Double.MAX_VALUE;
      RigidBodyBasics minimumDistanceBody = null;
      int minimumDistanceIndex = -1;

      for (int i = 0; i < rigidBodiesToCheck.length; i++)
      {
         List<Collidable> collidablesToCheck = collidableMap.get(rigidBodiesToCheck[i]);

         for (int j = 0; j < collidablesToCheck.size(); j++)
         {
            FrameShape3DReadOnly collisionShape = collidablesToCheck.get(j).getShape();
            pointToProject.changeFrame(collisionShape.getReferenceFrame());
            collisionShape.evaluatePoint3DCollision(pointToProject, contactPointPosition, surfaceNormal);

            pointToProject.changeFrame(contactPointPosition.getReferenceFrame());
            double distance = contactPointPosition.distance(pointToProject);
            if (distance < minimumProjectionDistance)
            {
               minimumProjectionDistance = distance;
               minimumDistanceBody = rigidBodiesToCheck[i];
               minimumDistanceIndex = j;
            }
         }
      }

      FrameShape3DReadOnly shapeToProjectTo = collidableMap.get(minimumDistanceBody).get(minimumDistanceIndex).getShape();
      pointToProject.changeFrame(shapeToProjectTo.getReferenceFrame());
      shapeToProjectTo.evaluatePoint3DCollision(pointToProject, contactPointPosition, surfaceNormal);

      return minimumDistanceBody;
   }

   public RigidBodyBasics[] getCollidableRigidBodies()
   {
      return collidableRigidBodies;
   }
}
