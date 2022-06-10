package us.ihmc.commonWalkingControlModules.contact.kinematics;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class FlatGroundContactDetector implements Updatable
{
   private static final double defaultContactThreshold = 0.03;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   protected StatusMessageOutputManager statusOutputManager;
   protected YoBoolean useAbsoluteGroundLocation = new YoBoolean("useAbsoluteGroundHeight", registry);
   protected YoDouble groundHeight = new YoDouble("groundHeight", registry);
   protected YoDouble contactThreshold = new YoDouble("contactThreshold", registry);

   protected final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   protected final List<ContactableShape> allCollidables = new ArrayList<>();
   protected final Map<RigidBodyBasics, List<ContactableShape>> contactableRigidBodyCollidables = new HashMap<>();
   protected final Map<RigidBodyBasics, List<DetectedContactPoint>> allContactPoints = new HashMap<>();

   private final MultiContactBalanceStatus multiContactBalanceStatus = new MultiContactBalanceStatus();

   public FlatGroundContactDetector(RigidBodyBasics rootBody,
                                    RobotCollisionModel collisionModel,
                                    YoGraphicsListRegistry graphicsListRegistry,
                                    List<String> collidableRigidBodies,
                                    YoRegistry parentRegistry)
   {
      contactThreshold.set(defaultContactThreshold);

      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(rootBody);
      Map<RigidBodyBasics, List<Collidable>> allCollidableMap = robotCollidables.stream().collect(Collectors.groupingBy(Collidable::getRigidBody));
      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = allRigidBodies.get(i);
         if (!collidableRigidBodies.contains(rigidBody.getName()))
            continue;

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

   @Override
   public void update(double time)
   {
      clearContacts();
      double flatGroundHeightThreshold = computeFlatGroundHeightThreshold();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<ContactableShape> rigidBodyCollidables = contactableRigidBodyCollidables.get(contactableRigidBodies.get(i));
         List<DetectedContactPoint> contactPoints = this.allContactPoints.get(contactableRigidBodies.get(i));

         List<FramePoint3DReadOnly> contactFramePoints = new ArrayList<>();
         for (int j = 0; j < rigidBodyCollidables.size(); j++)
         {
            rigidBodyCollidables.get(j).packContactPoints(contactFramePoints, flatGroundHeightThreshold);
         }

         for (int j = 0; j < contactFramePoints.size(); j++)
         {
            contactPoints.get(j).getContactPointPosition().set(contactFramePoints.get(j));
         }
      }

      if (statusOutputManager != null)
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

         statusOutputManager.reportStatusMessage(multiContactBalanceStatus);
      }
   }

   protected double computeFlatGroundHeightThreshold()
   {
      double groundHeight = allCollidables.stream().mapToDouble(ContactableShape::updateHeightInWorld).min().getAsDouble();
      if (useAbsoluteGroundLocation.getValue())
      {
         groundHeight = this.groundHeight.getValue();
      }
      double threshold = groundHeight + contactThreshold.getValue();
      return threshold;
   }

   public List<DetectedContactPoint> getContactPoints(RigidBodyBasics rigidBody)
   {
      return allContactPoints.get(rigidBody);
   }

   protected void clearContacts()
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

   public void setUseAbsoluteGroundLocation(boolean useAbsoluteGroundLocation)
   {
      this.useAbsoluteGroundLocation.set(useAbsoluteGroundLocation);
   }

   public void setGroundHeight(double groundHeight)
   {
      this.groundHeight.set(groundHeight);
   }

   public void setStatusOutputManager(StatusMessageOutputManager statusOutputManager)
   {
      this.statusOutputManager = statusOutputManager;
   }

   public void setContactThreshold(double contactThreshold)
   {
      this.contactThreshold.set(contactThreshold);
   }
}
