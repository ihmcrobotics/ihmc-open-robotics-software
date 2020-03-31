package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieKinematicsCollisionModel implements RobotCollisionModel
{
   private final DRCRobotJointMap jointMap;

   public ValkyrieKinematicsCollisionModel(DRCRobotJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      CollidableHelper helper = new CollidableHelper();
      List<Collidable> allCollidables = new ArrayList<>();
      allCollidables.addAll(setupArmCollisions(multiBodySystem, helper));
      allCollidables.addAll(setupNeckCollisions(multiBodySystem, helper));
      return allCollidables;
   }

   private List<Collidable> setupArmCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String bodyName = "Body";
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");
      SideDependentList<String> legNames = new SideDependentList<>("LeftLeg", "RightLeg");

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         Capsule3D torsoShapeTop = new Capsule3D(0.08, 0.15);
         torsoShapeTop.getPosition().set(0.1, 0.0, 0.01);
         torsoShapeTop.getAxis().set(Axis.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeTop, torso.getBodyFixedFrame()));
         Capsule3D torsoShapeBottom = new Capsule3D(0.1, 0.12);
         torsoShapeBottom.getPosition().set(0.09, 0.0, -0.14);
         torsoShapeBottom.setAxis(Axis.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottom, torso.getBodyFixedFrame()));

         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
         Capsule3D pelvisShapeTop = new Capsule3D(0.15, 0.1);
         pelvisShapeTop.setAxis(Axis.Y);
         pelvisShapeTop.getPosition().set(0.0, 0.0, -0.08);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShapeTop, pelvis.getParentJoint().getFrameAfterJoint()));
         Capsule3D pelvisShapeBottom = new Capsule3D(0.125, 0.14);
         pelvisShapeBottom.getPosition().set(0.0, 0.0, -0.155);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShapeBottom, pelvis.getParentJoint().getFrameAfterJoint()));

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hip = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem).getSuccessor();
            Sphere3D hipShape = new Sphere3D(0.125);
            hipShape.getPosition().set(-0.01, robotSide.negateIfLeftSide(0.01), -0.07);
            collidables.add(new Collidable(hip, collisionMask, collisionGroup, hipShape, hip.getBodyFixedFrame()));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         long collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(bodyName,
                                                          armNames.get(robotSide.getOppositeSide()),
                                                          legNames.get(RobotSide.LEFT),
                                                          legNames.get(RobotSide.RIGHT));

         RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
         JointBasics elbowRollJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);

         if (elbowRollJoint != null)
         {
            Capsule3D handShapeFist = new Capsule3D(0.04, 0.035);
            handShapeFist.getPosition().set(-0.01, robotSide.negateIfRightSide(0.033), -0.012);
            handShapeFist.getAxis().set(Axis.Z);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeFist, hand.getBodyFixedFrame()));
            Capsule3D handShapePalm = new Capsule3D(0.035, 0.04);
            handShapePalm.getPosition().set(-0.015, robotSide.negateIfLeftSide(0.01), -0.01);
            handShapePalm.getAxis().set(Axis.Z);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapePalm, hand.getBodyFixedFrame()));

            RigidBodyBasics forearm = elbowRollJoint.getSuccessor();
            Capsule3D forearmShape = new Capsule3D(0.21, 0.08);
            forearmShape.getPosition().setX(-0.02);
            forearmShape.getPosition().setY(robotSide.negateIfLeftSide(0.02));
            forearmShape.getPosition().setZ(-0.02);
            forearmShape.setAxis(Axis.Y);
            collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape, forearm.getBodyFixedFrame()));
         }
         else
         { // Assuming this val with arm mass sim
            RigidBodyBasics massSim = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem)
                                                         .getSuccessor();
            Capsule3D massSimShape = new Capsule3D(0.36, 0.065);
            massSimShape.getPosition().set(-0.015, robotSide.negateIfRightSide(0.04), -0.02);
            massSimShape.getAxis().set(Axis.Y);
            collidables.add(new Collidable(massSim, collisionMask, collisionGroup, massSimShape, massSim.getBodyFixedFrame()));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         long collisionMask = helper.getCollisionMask(legNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics upperLeg = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem).getSuccessor();
         Capsule3D upperLegShapeTop = new Capsule3D(0.25, 0.1);
         upperLegShapeTop.getPosition().set(0.015, 0.0, 0.10);
         upperLegShapeTop.setAxis(new Vector3D(-0.139, robotSide.negateIfLeftSide(0.1), 0.99));
         collidables.add(new Collidable(upperLeg, collisionMask, collisionGroup, upperLegShapeTop, upperLeg.getBodyFixedFrame()));
         Capsule3D upperLegShapeBottom = new Capsule3D(0.20, 0.105);
         upperLegShapeBottom.getPosition().set(0.01, robotSide.negateIfLeftSide(0.01), -0.10);
         upperLegShapeBottom.setAxis(new Vector3D(0.208, robotSide.negateIfRightSide(0.1), 0.978));
         collidables.add(new Collidable(upperLeg, collisionMask, collisionGroup, upperLegShapeBottom, upperLeg.getBodyFixedFrame()));

         RigidBodyBasics lowerLeg = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem).getSuccessor();
         Capsule3D lowerLegShape = new Capsule3D(0.23, 0.11);
         lowerLegShape.getPosition().set(0.01, robotSide.negateIfLeftSide(0.0025), 0.0);
         lowerLegShape.setAxis(new Vector3D(0.08, 0.0, 1.0));
         collidables.add(new Collidable(lowerLeg, collisionMask, collisionGroup, lowerLegShape, lowerLeg.getBodyFixedFrame()));
      }

      return collidables;
   }

   private List<Collidable> setupNeckCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String chinName = "Chin";
      String bodyName = "TorsoChin";

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(chinName);

         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         Capsule3D torsoShapeTop = new Capsule3D(0.15, 0.15);
         torsoShapeTop.getPosition().set(0.11, 0.0, -0.0);
         torsoShapeTop.getAxis().set(Axis.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeTop, torso.getBodyFixedFrame()));
      }

      { // Chin
         long collisionMask = helper.getCollisionMask(chinName);
         long collisionGroup = helper.createCollisionGroup(bodyName);

         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         Capsule3D chinShape = new Capsule3D(0.05, 0.02);
         chinShape.getPosition().set(0.07, 0.0, -0.17);
         chinShape.getAxis().set(Axis.Y);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, chinShape, head.getBodyFixedFrame()));
      }

      return collidables;
   }
}
