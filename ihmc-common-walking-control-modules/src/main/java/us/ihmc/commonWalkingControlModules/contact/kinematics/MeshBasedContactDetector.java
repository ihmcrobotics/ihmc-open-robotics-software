package us.ihmc.commonWalkingControlModules.contact.kinematics;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/**
 * Using simples collision shapes, performs contact detection for flat ground or convex polytopes.
 * Provides contact points and contact normals.
 */
public class MeshBasedContactDetector
{
   public static String graphicListRegistryName = "Kinematic-Detected Contact Points";
   private static final double coefficientOfFriction = 0.7;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private Consumer<MultiContactBalanceStatus> balanceStatusConsumer;
   private final YoDouble contactThreshold = new YoDouble("contactThreshold", registry);

   private final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   private final List<ContactableShape> allCollidables = new ArrayList<>();
   private final Map<RigidBodyBasics, List<ContactableShape>> contactableRigidBodyCollidables = new HashMap<>();
   private final Map<RigidBodyBasics, List<DetectedContactPoint>> allContactPoints = new HashMap<>();

   private List<FrameShape3DBasics> environmentShapes = null;
   private final MultiContactBalanceStatus previousMultiContactBalanceStatus = new MultiContactBalanceStatus();
   private final MultiContactBalanceStatus multiContactBalanceStatus = new MultiContactBalanceStatus();

   public MeshBasedContactDetector(RigidBodyBasics rootBody,
                                   RobotCollisionModel collisionModel)
   {
      this(rootBody, collisionModel, null, null);
   }

   public MeshBasedContactDetector(RigidBodyBasics rootBody,
                                   RobotCollisionModel collisionModel,
                                   YoGraphicsListRegistry graphicsListRegistry,
                                   YoRegistry parentRegistry)
   {
      contactThreshold.set(0.03);
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(rootBody);
      Map<RigidBodyBasics, List<Collidable>> allCollidableMap = robotCollidables.stream().collect(Collectors.groupingBy(Collidable::getRigidBody));

      for (RigidBodyBasics rigidBody : allCollidableMap.keySet())
      {
         contactableRigidBodies.add(rigidBody);
         List<Collidable> collidables = allCollidableMap.get(rigidBody);
         List<ContactableShape> rigidBodyCollidables = new ArrayList<>();

         for (int j = 0; j < collidables.size(); j++)
         {
            rigidBodyCollidables.add(new ContactableShape(collidables.get(j).getRigidBody().getName() + j, collidables.get(j)));
         }

         allCollidables.addAll(rigidBodyCollidables);
         contactableRigidBodyCollidables.put(rigidBody, rigidBodyCollidables);

         int maxNumberOfContactPoints = 0;
         for (int j = 0; j < collidables.size(); j++)
         {
            maxNumberOfContactPoints += ContactableShape.getMaximumNumberOfContactPoints(collidables.get(j));
         }

         List<DetectedContactPoint> contactPoints = new ArrayList<>();
         for (int j = 0; j < maxNumberOfContactPoints; j++)
         {
            DetectedContactPoint detectedContactPoint = new DetectedContactPoint(rigidBody.getName() + j, registry, graphicsListRegistry);
            detectedContactPoint.setVerticalContactNormal();
            contactPoints.add(detectedContactPoint);
         }

         this.allContactPoints.put(rigidBody, contactPoints);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public boolean update()
   {
      clearContactVisualization();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<ContactableShape> rigidBodyCollidables = contactableRigidBodyCollidables.get(contactableRigidBodies.get(i));
         List<FramePoint3DReadOnly> contactFramePoints = new ArrayList<>();
         List<FrameVector3DReadOnly> contactFrameNormals = new ArrayList<>();
         List<DetectedContactPoint> contactPoints = this.allContactPoints.get(contactableRigidBodies.get(i));

         for (int j = 0; j < rigidBodyCollidables.size(); j++)
         {
            detectEnvironmentContact(rigidBodyCollidables.get(j), contactFramePoints, contactFrameNormals);
         }

         for (int j = 0; j < contactFramePoints.size(); j++)
         {
            contactPoints.get(j).getContactPointPosition().set(contactFramePoints.get(j));
            contactPoints.get(j).getContactPointNormal().set(contactFrameNormals.get(j));
         }
      }

      updateBalanceStatus(multiContactBalanceStatus);

      boolean balanceStatusChanged = !previousMultiContactBalanceStatus.epsilonEquals(multiContactBalanceStatus, 1e-5);
      if (balanceStatusConsumer != null && balanceStatusChanged)
      {
         balanceStatusConsumer.accept(multiContactBalanceStatus);
      }

      previousMultiContactBalanceStatus.set(multiContactBalanceStatus);
      return balanceStatusChanged;
   }

   public List<RigidBodyBasics> getContactableRigidBodies()
   {
      return contactableRigidBodies;
   }

   public void updatePlaneContactState(List<YoPlaneContactState> contactStatesToUpdate)
   {
      for (YoPlaneContactState contactState : contactStatesToUpdate)
      {
         contactState.clear();
         contactState.setCoefficientOfFriction(coefficientOfFriction);

         List<DetectedContactPoint> detectedContactPoints = allContactPoints.get(contactState.getRigidBody());
         List<YoContactPoint> contactPoints = contactState.getContactPoints();

         for (int i = 0; i < detectedContactPoints.size(); i++)
         {
            DetectedContactPoint contactPoint = detectedContactPoints.get(i);
            if (!contactPoint.isInContact())
               break;

            contactState.setContactNormalVector(contactPoint.getContactPointNormal()); // TODO extract normal from DetectedContactPoint
            contactPoints.get(i).setInContact(true);
            contactPoints.get(i).setMatchingFrame(contactPoint.getContactPointPosition());
         }

         contactState.updateInContact();
      }
   }

   public Map<RigidBodyBasics, List<DetectedContactPoint>> getAllContactPoints()
   {
      return allContactPoints;
   }

   private boolean detectEnvironmentContact(ContactableShape contactableShape, List<FramePoint3DReadOnly> contactFramePoints, List<FrameVector3DReadOnly> contactNormals)
   {
      if (environmentShapes == null || environmentShapes.isEmpty())
      {
         return false;
      }

      BoundingBox3DReadOnly shapeBoundingBox = contactableShape.getShapeBoundingBox();
      contactableShape.update();

      for (int k = 0; k < environmentShapes.size(); k++)
      {
         FrameShape3DBasics environmentShape = environmentShapes.get(k);
         if (!shapeBoundingBox.intersectsEpsilon(environmentShape.getBoundingBox(), contactThreshold.getDoubleValue()))
            continue;

         if (contactableShape.detectEnvironmentContact(contactFramePoints, contactNormals, contactThreshold.getDoubleValue(), environmentShape))
         {
            return true;
         }
      }

      return false;
   }

   public void setEnvironmentShapes(List<FrameShape3DBasics> environmentShapes)
   {
      this.environmentShapes = environmentShapes;
   }

   public void updateBalanceStatus(MultiContactBalanceStatus multiContactBalanceStatus)
   {
      multiContactBalanceStatus.getContactPointsInWorld().clear();
      multiContactBalanceStatus.getSupportRigidBodyIds().clear();
      multiContactBalanceStatus.getSurfaceNormalsInWorld().clear();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<DetectedContactPoint> contactPoints = this.allContactPoints.get(contactableRigidBodies.get(i));

         for (int j = 0; j < contactPoints.size(); j++)
         {
            if (contactPoints.get(j).isInContact())
            {
               multiContactBalanceStatus.getSupportRigidBodyIds().add(rigidBody.hashCode());
               multiContactBalanceStatus.getContactPointsInWorld().add().set(contactPoints.get(j).getContactPointPosition());
               multiContactBalanceStatus.getSurfaceNormalsInWorld().add().set(contactPoints.get(j).getContactPointNormal());
            }
         }
      }
   }

   public List<DetectedContactPoint> getContactPoints(RigidBodyBasics rigidBody)
   {
      return allContactPoints.get(rigidBody);
   }

   protected void clearContactVisualization()
   {
      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<DetectedContactPoint> visualization = allContactPoints.get(rigidBody);
         for (int j = 0; j < visualization.size(); j++)
         {
            visualization.get(j).clearContact();
         }
      }
   }

   public void setContactThreshold(double contactThreshold)
   {
      this.contactThreshold.set(contactThreshold);
   }

   public void setBalanceStatusCallback(Consumer<MultiContactBalanceStatus> balanceStatusConsumer)
   {
      this.balanceStatusConsumer = balanceStatusConsumer;
   }

   public MultiContactBalanceStatus getMultiContactBalanceStatus()
   {
      return multiContactBalanceStatus;
   }
}
