package us.ihmc.avatar.contactEstimation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ContactPointParticle;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.MeshSurfaceProjector;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.List;

public abstract class AvatarMeshProjectionVisualizer
{
   private static final boolean PRINT_PROJECTIONS = true;
   private static final boolean PROJECT_TO_SPECIFIC_LINK = true;

   public AvatarMeshProjectionVisualizer()
   {
      DRCRobotModel robotModel = getRobotModel();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      List<Pair<RigidBodyBasics, FramePoint3D>> queryPoints = createListOfPointsToProject(fullRobotModel);

      RobotCollisionModel collisionModel = getCollisionModel();
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());
      MeshSurfaceProjector meshSurfaceProjector = new MeshSurfaceProjector(robotCollidables);

      for (Pair<RigidBodyBasics, FramePoint3D> query : queryPoints)
      {
         RigidBodyBasics rigidBody = query.getLeft();
         FramePoint3D queryPoint = query.getRight();

         JointBasics[] orderedJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
         ContactPointParticle contactPointParticle = new ContactPointParticle("", orderedJoints);
         contactPointParticle.setRigidBody(rigidBody);

         if (PROJECT_TO_SPECIFIC_LINK)
         {
            meshSurfaceProjector.projectToSpecificLink(queryPoint, contactPointParticle.getContactPointPosition(), contactPointParticle.getSurfaceNormal(), rigidBody);

            if (PRINT_PROJECTIONS)
            {
               FramePoint3D position = contactPointParticle.getContactPointPosition();
               position.changeFrame(rigidBody.getParentJoint().getFrameAfterJoint());
               System.out.println(rigidBody.getParentJoint().getName() + "\t" + EuclidCoreIOTools.getTuple3DString(position));
            }
         }
         else
         {
            RigidBodyBasics closestLink = meshSurfaceProjector.projectToClosestLink(queryPoint,
                                                                                    contactPointParticle.getContactPointPosition(),
                                                                                    contactPointParticle.getSurfaceNormal());
            contactPointParticle.setRigidBody(closestLink);

            if (PRINT_PROJECTIONS)
            {
               FramePoint3D position = contactPointParticle.getContactPointPosition();
               position.changeFrame(closestLink.getParentJoint().getFrameAfterJoint());
               System.out.println(closestLink.getParentJoint().getName() + "\t" + EuclidCoreIOTools.getTuple3DString(position));
            }
         }

         contactPointParticle.update();

         graphics3DObject.identity();

         queryPoint.changeFrame(ReferenceFrame.getWorldFrame());
         graphics3DObject.translate(queryPoint);
         graphics3DObject.addSphere(0.012);

         graphics3DObject.identity();
         graphics3DObject.translate(contactPointParticle.getContactPointPosition());
         graphics3DObject.addSphere(0.012);

         graphics3DObject.identity();

         RigidBodyTransform transformToWorldFrame = contactPointParticle.getContactPointFrame().getTransformToWorldFrame();

         graphics3DObject.transform(transformToWorldFrame);
         graphics3DObject.addCoordinateSystem(0.1);
      }

      FloatingRootJointRobot scsRobot = robotModel.createHumanoidFloatingRootJointRobot(true);
      scsRobot.setPositionInWorld(new Vector3D());

      AppearanceDefinition collisionAppearance = YoAppearance.DarkGreen();
      collisionAppearance.setTransparency(0.5);
      addKinematicsCollisionGraphics(fullRobotModel.getElevator(), scsRobot, collisionModel, collisionAppearance);

      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobot);
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   protected abstract DRCRobotModel getRobotModel();

   protected abstract List<Pair<RigidBodyBasics, FramePoint3D>> createListOfPointsToProject(FullHumanoidRobotModel fullRobotModel);

   protected abstract RobotCollisionModel getCollisionModel();

   public static void addKinematicsCollisionGraphics(RigidBodyBasics rootBody, Robot robot, RobotCollisionModel collisionModel, AppearanceDefinition appearance)
   {
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(rootBody);

      for (Collidable collidable : robotCollidables)
      {
         Link link = robot.getLink(collidable.getRigidBody().getName());
         if (link.getLinkGraphics() == null)
         {
            link.setLinkGraphics(getGraphics(collidable, appearance));
         }
         else
         {
            link.getLinkGraphics().combine(getGraphics(collidable, appearance));
         }
      }
   }

   private static Graphics3DObject getGraphics(Collidable collidable, AppearanceDefinition appearance)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape().getReferenceFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(transformToParentJoint);

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), transform.getRotation());
         transform.getTranslation().set(capsule.getPosition());
         graphics.transform(transform);
         graphics.addCapsule(capsule.getRadius(),
                             capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                             appearance);
      }
      else if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         graphics.translate(box.getPosition());
         graphics.rotate(new RotationMatrix(box.getOrientation()));
         graphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, appearance);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         graphics.translate(pointShape);
         graphics.addSphere(0.01, appearance);
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly)
      {
         FrameEllipsoid3DReadOnly ellipsoidShape = (FrameEllipsoid3DReadOnly) shape;
         graphics.transform(new RigidBodyTransform(ellipsoidShape.getPose()));
         graphics.addEllipsoid(ellipsoidShape.getRadiusX(), ellipsoidShape.getRadiusY(), ellipsoidShape.getRadiusZ(), appearance);
      }
      else if (shape instanceof Cylinder3DReadOnly)
      {
         Cylinder3DReadOnly cylinder = (Cylinder3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder.getAxis(), transform.getRotation());
         transform.getTranslation().set(cylinder.getPosition());
         graphics.transform(transform);
         graphics.translate(0.0, 0.0, -0.5 * cylinder.getLength());
         graphics.addCylinder(cylinder.getLength(), cylinder.getRadius(), appearance);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }
}
