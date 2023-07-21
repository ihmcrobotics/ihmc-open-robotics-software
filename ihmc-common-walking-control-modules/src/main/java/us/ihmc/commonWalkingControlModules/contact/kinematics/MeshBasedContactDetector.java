package us.ihmc.commonWalkingControlModules.contact.kinematics;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Using simples collision shapes, performs contact detection for flat ground or convex polytopes.
 * Provides contact points and contact normals.
 */
public class MeshBasedContactDetector
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private double contactThreshold = 0.02;
   private final BoundingBox3D shapeBoundingBoxInWorld = new BoundingBox3D();
   private List<FrameShape3DBasics> environmentShapes = null;

   private final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   private final Map<RigidBodyBasics, List<Collidable>> collidableMap;
   private final Map<RigidBodyBasics, List<YoDetectedContactPoint>> contactPointMap = new HashMap<>();

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
   private final RigidBodyTransform robotToEnvironmentTransform = new RigidBodyTransform();
   private final RigidBodyTransform environmentToRobotTransform = new RigidBodyTransform();

   private final FramePoint3D tempContactPointA = new FramePoint3D();
   private final FramePoint3D tempContactPointB = new FramePoint3D();
   private final FrameVector3D tempSurfaceNormal = new FrameVector3D();

   public MeshBasedContactDetector(List<Collidable> robotCollidables)
   {
      collidableMap = robotCollidables.stream().collect(Collectors.groupingBy(Collidable::getRigidBody));

      for (RigidBodyBasics rigidBody : collidableMap.keySet())
      {
         contactableRigidBodies.add(rigidBody);
         List<Collidable> collidables = collidableMap.get(rigidBody);

         List<YoDetectedContactPoint> contactPoints = new ArrayList<>();
         for (int j = 0; j < collidables.size(); j++)
         {
            YoDetectedContactPoint detectedContactPoint = new YoDetectedContactPoint(rigidBody.getName() + j, registry);
            detectedContactPoint.setVerticalContactNormal();
            contactPoints.add(detectedContactPoint);
         }

         this.contactPointMap.put(rigidBody, contactPoints);
      }
   }

   public void update()
   {
      /* Reset all contact points */
      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<YoDetectedContactPoint> visualization = contactPointMap.get(rigidBody);
         for (int j = 0; j < visualization.size(); j++)
         {
            visualization.get(j).clearContact();
         }
      }

      /* Check that environment is set */
      if (environmentShapes == null || environmentShapes.isEmpty())
      {
         return;
      }

      /* Do kinematic contact detection */
      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<Collidable> collidables = collidableMap.get(contactableRigidBodies.get(i));
         List<YoDetectedContactPoint> contactPoints = contactPointMap.get(contactableRigidBodies.get(i));
         int contactPointsDetected = 0;

         for (int j = 0; j < collidables.size(); j++)
         {
            Collidable robotCollidable = collidables.get(j);

            robotCollidable.getShape().getReferenceFrame().update();
            robotCollidable.getShape().getBoundingBox(ReferenceFrame.getWorldFrame(), shapeBoundingBoxInWorld);

            for (int k = 0; k < environmentShapes.size(); k++)
            {
               FrameShape3DBasics environmentShape = environmentShapes.get(k);

               if (!shapeBoundingBoxInWorld.intersectsEpsilon(environmentShape.getBoundingBox(), contactThreshold))
               { // check bounding boxes
                  continue;
               }

               if (doCollisionCheck(robotCollidable.getShape(), environmentShape, contactPoints.get(contactPointsDetected)))
               { // full collision check
                  contactPointsDetected++;
                  break;
               }
            }
         }
      }
   }

   private boolean doCollisionCheck(FrameShape3DReadOnly robotShape, FrameShape3DBasics environmentShape, YoDetectedContactPoint contactPointToSet)
   {
      // Robot collidables are by default expressed in a frame fixed to the rigid body.
      // Since Collidable has a ReadOnly interface for the shape, the environment shape is transformed into the frame of the robot collidable for collision detection
      // If a collision is detected, it's transformed to world frame and stored

      ReferenceFrame robotShapeFrame = robotShape.getReferenceFrame();
      ReferenceFrame environmentShapeFrame = environmentShape.getReferenceFrame();

      robotShapeFrame.getTransformToDesiredFrame(robotToEnvironmentTransform, environmentShapeFrame);
      environmentToRobotTransform.setAndInvert(robotToEnvironmentTransform);

      environmentShape.applyTransform(environmentToRobotTransform);
      environmentShape.setReferenceFrame(robotShapeFrame);

      collisionDetector.evaluateCollision(environmentShape, robotShape, collisionResult);

      boolean contactDetected = collisionResult.getSignedDistance() < contactThreshold;
      if (contactDetected)
      {
         tempContactPointA.setIncludingFrame(robotShapeFrame, collisionResult.getPointOnB());
         environmentShape.evaluatePoint3DCollision(tempContactPointA, tempContactPointB, tempSurfaceNormal);

         tempContactPointA.setReferenceFrame(robotShapeFrame);
         tempSurfaceNormal.setReferenceFrame(robotShapeFrame);
         tempContactPointA.changeFrame(ReferenceFrame.getWorldFrame());
         tempSurfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

         contactPointToSet.getContactPointPosition().set(tempContactPointA);
         contactPointToSet.getContactPointNormal().set(tempSurfaceNormal);
      }

      environmentShape.applyTransform(robotToEnvironmentTransform);
      environmentShape.setReferenceFrame(environmentShapeFrame);

      return contactDetected;
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<YoDetectedContactPoint> yoContactPoints = contactPointMap.get(contactableRigidBodies.get(i));
         for (int j = 0; j < yoContactPoints.size(); j++)
         {
            group.addChild(yoContactPoints.get(j).getSCS2Graphic());
         }
      }

      return group;
   }

   public List<RigidBodyBasics> getContactableRigidBodies()
   {
      return contactableRigidBodies;
   }

   public Map<RigidBodyBasics, List<YoDetectedContactPoint>> getContactPointMap()
   {
      return contactPointMap;
   }

   public void setEnvironmentShapes(List<FrameShape3DBasics> environmentShapes)
   {
      this.environmentShapes = environmentShapes;
   }

   public List<YoDetectedContactPoint> getContactPoints(RigidBodyBasics rigidBody)
   {
      return contactPointMap.get(rigidBody);
   }

   public void setContactThreshold(double contactThreshold)
   {
      this.contactThreshold = contactThreshold;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public void packMultiContactBalanceStatus(MultiContactBalanceStatus balanceStatus)
   {
      balanceStatus.getContactPointsInWorld().clear();
      balanceStatus.getSurfaceNormalsInWorld().clear();
      balanceStatus.getSupportRigidBodyIds().clear();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<YoDetectedContactPoint> contactPoints = contactPointMap.get(rigidBody);

         for (int j = 0; j < contactPoints.size(); j++)
         {
            YoDetectedContactPoint contactPoint = contactPoints.get(j);
            if (contactPoint.isInContact())
            {
               balanceStatus.getContactPointsInWorld().add().set(contactPoint.getContactPointPosition());
               balanceStatus.getSurfaceNormalsInWorld().add().set(contactPoint.getContactPointNormal());
               balanceStatus.getSupportRigidBodyIds().add(rigidBody.hashCode());
            }
         }
      }
   }
}
