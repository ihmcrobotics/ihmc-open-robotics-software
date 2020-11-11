package us.ihmc.valkyrie.externalForceEstimation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ContactPointParticle;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ContactPointProjector;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.valkyrie.ValkyrieSDFLoadingDemo.addKinematicsCollisionGraphics;

public class ValkyrieContactPointProjectorTest
{
   public ValkyrieContactPointProjectorTest()
   {
      boolean projectToSpecificLink = true;
      boolean printProjectedLocation = true;

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());
      ContactPointProjector contactPointProjector = new ContactPointProjector(robotCollidables);

      List<Pair<RigidBodyBasics, FramePoint3D>> queryPoints = new ArrayList<>();

      RigidBodyBasics chest = fullRobotModel.getChest();
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.1, 0.2)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, 0.0)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.05, 0.1)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.05, 0.4)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, -0.3, 0.3)));

      RigidBodyBasics leftForearmLink = fullRobotModel.getOneDoFJointByName("leftForearmYaw").getSuccessor();
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.6, 0.4)));
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.7, 0.2)));
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, 0.75, 0.2)));

      RigidBodyBasics leftHand = fullRobotModel.getHand(RobotSide.LEFT);
      queryPoints.add(Pair.of(leftHand, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.95, 0.35)));

      RigidBodyBasics rightForearmLink = fullRobotModel.getOneDoFJointByName("rightForearmYaw").getSuccessor();
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.6, 0.4)));
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.7, 0.2)));
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, -0.75, 0.2)));

      RigidBodyBasics rightHand = fullRobotModel.getHand(RobotSide.RIGHT);
      queryPoints.add(Pair.of(rightHand, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.9, 0.2)));

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, -0.1)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.3, -0.15)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, 0.1, -0.2)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, -0.3)));

      RigidBodyBasics leftHipPitchLink = fullRobotModel.getOneDoFJointByName("leftHipPitch").getSuccessor();
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.5, -0.3)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.5, -0.35)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.5, -0.4)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, 0.3, -0.45)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.35, -0.5)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.45, -0.55)));

      RigidBodyBasics rightHipPitchLink = fullRobotModel.getOneDoFJointByName("rightHipPitch").getSuccessor();
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.5, -0.3)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.5, -0.35)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, -0.5, -0.4)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, -0.3, -0.45)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, -0.35, -0.5)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.45, -0.55)));

      RigidBodyBasics leftKneePitchLink = fullRobotModel.getOneDoFJointByName("leftKneePitch").getSuccessor();
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.2, -0.7)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.4, -0.75)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.4, -0.8)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.3, -0.85)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.1, -1.1)));

      RigidBodyBasics rightKneePitchLink = fullRobotModel.getOneDoFJointByName("rightKneePitch").getSuccessor();
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.2, -0.7)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.4, -0.75)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, -0.4, -0.8)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.3, -0.85)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.1, -1.1)));

      Graphics3DObject graphics3DObject = new Graphics3DObject();

      for (Pair<RigidBodyBasics, FramePoint3D> query : queryPoints)
      {
         RigidBodyBasics rigidBody = query.getLeft();
         FramePoint3D queryPoint = query.getRight();

         JointBasics[] orderedJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
         ContactPointParticle contactPointParticle = new ContactPointParticle("", orderedJoints);
         contactPointParticle.setRigidBody(rigidBody);

         if (projectToSpecificLink)
         {
            contactPointProjector.projectToSpecificLink(queryPoint, contactPointParticle.getContactPointPosition(), contactPointParticle.getSurfaceNormal(), rigidBody);

            if (printProjectedLocation)
            {
               FramePoint3D position = contactPointParticle.getContactPointPosition();
               position.changeFrame(rigidBody.getParentJoint().getFrameAfterJoint());
               System.out.println(rigidBody.getParentJoint().getName() + "\t" + EuclidCoreIOTools.getTuple3DString(position));
            }
         }
         else
         {
            RigidBodyBasics closestLink = contactPointProjector.projectToClosestLink(queryPoint,
                                                                                     contactPointParticle.getContactPointPosition(),
                                                                                     contactPointParticle.getSurfaceNormal());
            contactPointParticle.setRigidBody(closestLink);

            if (printProjectedLocation)
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

      FloatingRootJointRobot valkyrieRobot = robotModel.createHumanoidFloatingRootJointRobot(true);
      valkyrieRobot.setPositionInWorld(new Vector3D());
      addKinematicsCollisionGraphics(fullRobotModel, valkyrieRobot, robotModel.getHumanoidRobotKinematicsCollisionModel());

      SimulationConstructionSet scs = new SimulationConstructionSet(valkyrieRobot);
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ValkyrieContactPointProjectorTest();
   }
}
