package us.ihmc.commonWalkingControlModules.contact.kinematics;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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

/**
 * Using simples collision shapes, performs contact detection for flat ground or planar regions.
 * Provides contact points and contact normals.
 */
public class MeshBasedContactDetector
{
   private static final double defaultContactThreshold = 0.03;
   private static final double coefficientOfFriction = 0.7;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   protected StatusMessageOutputManager statusOutputManager;
   protected YoBoolean useAbsoluteGroundLocation = new YoBoolean("useAbsoluteGroundHeight", registry);
   protected YoDouble groundHeight = new YoDouble("groundHeight", registry);
   protected YoDouble contactThreshold = new YoDouble("contactThreshold", registry);

   protected final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   protected final List<ContactableShape> allCollidables = new ArrayList<>();
   protected final Map<RigidBodyBasics, List<ContactableShape>> contactableRigidBodyCollidables = new HashMap<>();
   protected final Map<RigidBodyBasics, List<DetectedContactPoint>> allContactPoints = new HashMap<>();

   private PlanarRegionsList planarRegionsList = null;
   private final HashMap<PlanarRegion, ConvexPolytope3DReadOnly> contactableVolumeMap = new HashMap<>();
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
      contactThreshold.set(defaultContactThreshold);

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

   public int getMaxNumberOfContactPointsPerBody()
   {
      return 4;
   }

   public void update()
   {
      clearContacts();
      double flatGroundHeightThreshold = computeFlatGroundHeightThreshold();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<ContactableShape> rigidBodyCollidables = contactableRigidBodyCollidables.get(contactableRigidBodies.get(i));
         List<FramePoint3DReadOnly> contactFramePoints = new ArrayList<>();
         List<DetectedContactPoint> contactPoints = this.allContactPoints.get(contactableRigidBodies.get(i));

         /* Limit to one plane contact per rigid body */
         Vector3D contactNormal = new Vector3D();

         for (int j = 0; j < rigidBodyCollidables.size(); j++)
         {
            if (rigidBodyCollidables.get(j).detectFlatGroundContact(contactFramePoints, flatGroundHeightThreshold))
            {
               contactNormal.set(Axis3D.Z);
               break;
            }
            else if (detectPlanarRegionContact(rigidBodyCollidables.get(j), contactFramePoints, contactNormal))
            {
               break;
            }
         }

         for (int j = 0; j < contactFramePoints.size(); j++)
         {
            contactPoints.get(j).getContactPointPosition().set(contactFramePoints.get(j));
            contactPoints.get(j).getContactPointNormal().set(contactNormal);
         }
      }

      if (statusOutputManager != null)
      {
         updateBalanceStatus(multiContactBalanceStatus);
         statusOutputManager.reportStatusMessage(multiContactBalanceStatus);
      }
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

   private boolean detectPlanarRegionContact(ContactableShape contactableShape, List<FramePoint3DReadOnly> contactFramePoints, Vector3D contactNormal)
   {
      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return false;
      }

      BoundingBox3DReadOnly shapeBoundingBox = contactableShape.getShapeBoundingBox();

      for (int k = 0; k < planarRegionsList.getNumberOfPlanarRegions(); k++)
      {
         ConvexPolytope3DReadOnly contactablePolytope = contactableVolumeMap.get(planarRegionsList.getPlanarRegion(k));
         if (!shapeBoundingBox.intersectsExclusive(contactablePolytope.getBoundingBox()))
            continue;

         if (contactableShape.detectPolytopeContact(contactFramePoints, contactThreshold.getDoubleValue(), contactablePolytope))
         {
            contactNormal.set(planarRegionsList.getPlanarRegion(k).getNormal());
            FramePoint3D bodyFixedFrame = new FramePoint3D(contactableShape.getCollidable().getRigidBody().getBodyFixedFrame());
            bodyFixedFrame.changeFrame(ReferenceFrame.getWorldFrame());
            bodyFixedFrame.applyTransform(planarRegionsList.getPlanarRegion(k).getTransformToLocal());
            if (bodyFixedFrame.getZ() < 0.0)
               contactNormal.negate();
            return true;
         }
      }

      return false;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      contactableVolumeMap.clear();

      if (planarRegionsList == null)
         return;

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = planarRegionsList.getPlanarRegion(i);
         ConvexPolytope3D contactablePolytope = new ConvexPolytope3D();

         ConvexPolygon2D convexHull = region.getConvexHull();
         for (double sign : new double[]{-1.0, 1.0})
         {
            for (int j = 0; j < convexHull.getNumberOfVertices(); j++)
            {
               Point3D vertex = new Point3D(convexHull.getVertex(j).getX(), convexHull.getVertex(j).getY(), sign * contactThreshold.getValue());
               vertex.applyTransform(region.getTransformToWorld());
               contactablePolytope.addVertex(vertex);
            }
         }
         contactableVolumeMap.put(region, contactablePolytope);
      }
   }

   public HashMap<PlanarRegion, ConvexPolytope3DReadOnly> getContactableVolumeMap()
   {
      return contactableVolumeMap;
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
