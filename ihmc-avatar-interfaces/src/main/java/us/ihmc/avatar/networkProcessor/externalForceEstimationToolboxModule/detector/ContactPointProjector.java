package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.EstimatorContactPoint;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class ContactPointProjector
{
   private final List<Collidable> allCollidables;
   private final List<Collidable> collidablesToCheck = new ArrayList<>();

   public ContactPointProjector(List<Collidable> allCollidables)
   {
      this.allCollidables = allCollidables;
   }

   public void initialize(RigidBodyBasics rigidBody)
   {
      collidablesToCheck.clear();
      allCollidables.stream().filter(collidable -> collidable.getRigidBody() == rigidBody).forEach(collidablesToCheck::add);

      if (collidablesToCheck.isEmpty())
      {
         throw new RuntimeException("The rigid body " + rigidBody + " has no collidables");
      }
   }

   public boolean isPointInside(FramePoint3DBasics queryPoint)
   {
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

   public void computeProjection(FramePoint3DBasics pointToProject, EstimatorContactPoint contactPointToSet)
   {
      double minimumProjectionDistance = Double.MAX_VALUE;
      int minimumDistanceIndex = -1;

      FramePoint3D contactPointPosition = contactPointToSet.getContactPointPosition();
      FrameVector3D surfaceNormal = contactPointToSet.getSurfaceNormal();

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

      contactPointToSet.update();
   }

   public List<Collidable> getCollidablesToCheck()
   {
      return collidablesToCheck;
   }
}
