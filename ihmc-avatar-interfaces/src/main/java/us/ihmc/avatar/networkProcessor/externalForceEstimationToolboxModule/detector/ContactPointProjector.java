package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

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

   private final FramePoint3D surfacePoint = new FramePoint3D();
   private final FrameVector3D surfaceNormal = new FrameVector3D();
   private final FramePose3D surfacePose = new FramePose3D();
   private final PoseReferenceFrame surfaceFrame = new PoseReferenceFrame("surfaceFrame", ReferenceFrame.getWorldFrame());

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

   public void computeProjection(FramePoint3DBasics pointToProject)
   {
      double minimumProjectionDistance = Double.MAX_VALUE;
      int minimumDistanceIndex = -1;

      for (int i = 0; i < collidablesToCheck.size(); i++)
      {
         FrameShape3DReadOnly collisionShape = collidablesToCheck.get(i).getShape();
         pointToProject.changeFrame(collisionShape.getReferenceFrame());
         collisionShape.evaluatePoint3DCollision(pointToProject, surfacePoint, surfaceNormal);

         boolean isInsideOthers = false;
         for (int j = i + 1; j < collidablesToCheck.size(); j++)
         {
            FrameShape3DReadOnly otherCollisionShape = collidablesToCheck.get(j).getShape();
            pointToProject.changeFrame(otherCollisionShape.getReferenceFrame());

            if (otherCollisionShape.isPointInside(surfacePoint))
            {
               isInsideOthers = true;
               break;
            }
         }

         if (isInsideOthers)
         {
            continue;
         }

         pointToProject.changeFrame(surfacePoint.getReferenceFrame());
         double distance = surfacePoint.distance(pointToProject);
         if (distance < minimumProjectionDistance)
         {
            minimumProjectionDistance = distance;
            minimumDistanceIndex = i;
         }
      }

      FrameShape3DReadOnly shapeToProjectTo = collidablesToCheck.get(minimumDistanceIndex).getShape();
      pointToProject.changeFrame(shapeToProjectTo.getReferenceFrame());
      shapeToProjectTo.evaluatePoint3DCollision(pointToProject, surfacePoint, surfaceNormal);

      surfacePoint.changeFrame(ReferenceFrame.getWorldFrame());
      surfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

      surfacePose.getPosition().set(surfacePoint);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, surfacePose.getOrientation());
      surfaceFrame.setPoseAndUpdate(surfacePose);
   }

   public FramePoint3D getSurfacePoint()
   {
      return surfacePoint;
   }

   public FrameVector3D getSurfaceNormal()
   {
      return surfaceNormal;
   }

   public PoseReferenceFrame getSurfaceFrame()
   {
      return surfaceFrame;
   }
}
