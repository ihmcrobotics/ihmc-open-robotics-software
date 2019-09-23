package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidableHelper;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieKinematicsCollisionModel implements HumanoidRobotKinematicsCollisionModel
{
   public ValkyrieKinematicsCollisionModel()
   {
   }

   @Override
   public List<KinematicsCollidable> getRobotCollidables(FullHumanoidRobotModel fullRobotModel)
   {
      KinematicsCollidableHelper helper = new KinematicsCollidableHelper();
      List<KinematicsCollidable> allCollidables = new ArrayList<>();
      allCollidables.addAll(setupArmCollisions(fullRobotModel, helper));
      allCollidables.addAll(setupNeckCollisions(fullRobotModel, helper));
      return allCollidables;
   }

   private List<KinematicsCollidable> setupArmCollisions(FullHumanoidRobotModel fullRobotModel, KinematicsCollidableHelper helper)
   {
      List<KinematicsCollidable> collidables = new ArrayList<>();

      double minimumSafeDistance = 0.0;

      String bodyName = "Body";
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");
      SideDependentList<String> legNames = new SideDependentList<>("LeftLeg", "RightLeg");

      { // Body
         int collisionMask = helper.getCollisionMask(bodyName);
         int collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics torso = fullRobotModel.getChest();
         Capsule3D torsoShapeTop = new Capsule3D(0.08, 0.15);
         torsoShapeTop.getPosition().set(0.1, 0.0, 0.01);
         torsoShapeTop.getAxis().set(Axis.Y);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeTop, torso.getBodyFixedFrame(), minimumSafeDistance));
         Capsule3D torsoShapeBottom = new Capsule3D(0.1, 0.12);
         torsoShapeBottom.getPosition().set(0.09, 0.0, -0.14);
         torsoShapeBottom.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeBottom, torso.getBodyFixedFrame(), minimumSafeDistance));

         RigidBodyBasics pelvis = fullRobotModel.getPelvis();
         Capsule3D pelvisShapeTop = new Capsule3D(0.15, 0.1);
         pelvisShapeTop.setAxis(Axis.Y);
         pelvisShapeTop.getPosition().set(0.0, 0.0, -0.08);
         collidables.add(new KinematicsCollidable(pelvis, collisionMask, collisionGroup, pelvisShapeTop, pelvis.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));
         Capsule3D pelvisShapeBottom = new Capsule3D(0.125, 0.14);
         pelvisShapeBottom.getPosition().set(0.0, 0.0, -0.155);
         collidables.add(new KinematicsCollidable(pelvis, collisionMask, collisionGroup, pelvisShapeBottom, pelvis.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hip = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).getSuccessor();
            Sphere3D hipShape = new Sphere3D(0.125);
            hipShape.getPosition().set(-0.01, robotSide.negateIfLeftSide(0.01), -0.07);
            collidables.add(new KinematicsCollidable(hip, collisionMask, collisionGroup, hipShape, hip.getBodyFixedFrame(), minimumSafeDistance));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         int collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         int collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()), legNames.get(RobotSide.LEFT), legNames.get(RobotSide.RIGHT));

         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         Capsule3D handShapeFist = new Capsule3D(0.04, 0.035);
         handShapeFist.getPosition().set(-0.01, robotSide.negateIfRightSide(0.033), -0.012);
         handShapeFist.getAxis().set(Axis.Z);
         collidables.add(new KinematicsCollidable(hand, collisionMask, collisionGroup, handShapeFist, hand.getBodyFixedFrame(), minimumSafeDistance));
         Capsule3D handShapePalm = new Capsule3D(0.035, 0.04);
         handShapePalm.getPosition().set(-0.015, robotSide.negateIfLeftSide(0.01), -0.01);
         handShapePalm.getAxis().set(Axis.Z);
         collidables.add(new KinematicsCollidable(hand, collisionMask, collisionGroup, handShapePalm, hand.getBodyFixedFrame(), minimumSafeDistance));

         RigidBodyBasics forearm = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL).getSuccessor();
         Capsule3D forearmShape = new Capsule3D(0.21, 0.08);
         forearmShape.getPosition().setX(-0.02);
         forearmShape.getPosition().setY(robotSide.negateIfLeftSide(0.02));
         forearmShape.getPosition().setZ(-0.02);
         forearmShape.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(forearm, collisionMask, collisionGroup, forearmShape, forearm.getBodyFixedFrame(), minimumSafeDistance));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         int collisionMask = helper.getCollisionMask(legNames.get(robotSide));
         int collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics upperLeg = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getSuccessor();
         Capsule3D upperLegShapeTop = new Capsule3D(0.25, 0.1);
         upperLegShapeTop.getPosition().set(0.015, 0.0, 0.10);
         upperLegShapeTop.setAxis(new Vector3D(-0.139, robotSide.negateIfLeftSide(0.1), 0.99));
         collidables.add(new KinematicsCollidable(upperLeg, collisionMask, collisionGroup, upperLegShapeTop, upperLeg.getBodyFixedFrame(), minimumSafeDistance));
         Capsule3D upperLegShapeBottom = new Capsule3D(0.20, 0.105);
         upperLegShapeBottom.getPosition().set(0.01, robotSide.negateIfLeftSide(0.01), -0.10);
         upperLegShapeBottom.setAxis(new Vector3D(0.208, robotSide.negateIfRightSide(0.1), 0.978));
         collidables.add(new KinematicsCollidable(upperLeg, collisionMask, collisionGroup, upperLegShapeBottom, upperLeg.getBodyFixedFrame(), minimumSafeDistance));

         RigidBodyBasics lowerLeg = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getSuccessor();
         Capsule3D lowerLegShape = new Capsule3D(0.23, 0.11);
         lowerLegShape.getPosition().set(0.01, robotSide.negateIfLeftSide(0.0025), 0.0);
         lowerLegShape.setAxis(new Vector3D(0.08, 0.0, 1.0));
         collidables.add(new KinematicsCollidable(lowerLeg, collisionMask, collisionGroup, lowerLegShape, lowerLeg.getBodyFixedFrame(), minimumSafeDistance));
      }

      return collidables;
   }

   private List<KinematicsCollidable> setupNeckCollisions(FullHumanoidRobotModel fullRobotModel, KinematicsCollidableHelper helper)
   {
      List<KinematicsCollidable> collidables = new ArrayList<>();

      double minimumSafeDistance = 0.0;

      String chinName = "Chin";
      String bodyName = "TorsoChin";

      { // Body
         int collisionMask = helper.getCollisionMask(bodyName);
         int collisionGroup = helper.createCollisionGroup(chinName);

         RigidBodyBasics torso = fullRobotModel.getChest();
         Capsule3D torsoShapeTop = new Capsule3D(0.15, 0.15);
         torsoShapeTop.getPosition().set(0.09, 0.0, -0.02);
         torsoShapeTop.getAxis().set(Axis.Y);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeTop, torso.getBodyFixedFrame(), minimumSafeDistance));
      }

      { // Chin
         int collisionMask = helper.getCollisionMask(chinName);
         int collisionGroup = helper.createCollisionGroup(bodyName);

         RigidBodyBasics head = fullRobotModel.getHead();
         Capsule3D chinShape = new Capsule3D(0.05, 0.02);
         chinShape.getPosition().set(0.07, 0.0, -0.17);
         chinShape.getAxis().set(Axis.Y);
         collidables.add(new KinematicsCollidable(head, collisionMask, collisionGroup, chinShape, head.getBodyFixedFrame(), minimumSafeDistance));
      }

      return collidables;
   }
}
