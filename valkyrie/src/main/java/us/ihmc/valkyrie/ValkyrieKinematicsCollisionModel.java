package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidableHelper;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.RotationMatrixTools;
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
      List<KinematicsCollidable> collidables = new ArrayList<>();

      double minimumSafeDistance = 0.0;

      String bodyName = "Body";
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");
      SideDependentList<String> legNames = new SideDependentList<>("LeftLeg", "RightLeg");

      { // Body
         int collisionMask = helper.getCollisionMask(bodyName);
         int collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics torso = fullRobotModel.getChest();
         Capsule3D torsoShapeMain = new Capsule3D(0.17, 0.20);
         RotationMatrixTools.applyPitchRotation(Math.toRadians(15), torsoShapeMain.getAxis(), torsoShapeMain.getAxis());
         torsoShapeMain.getPosition().set(-0.06, 0.0, 0.205);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeMain, torso.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));
         Capsule3D torsoShapeBreast = new Capsule3D(0.16, 0.1);
         torsoShapeBreast.setAxis(Axis.Y);
         torsoShapeBreast.getPosition().set(0.06, 0.0, 0.25);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeBreast, torso.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));

         RigidBodyBasics pelvis = fullRobotModel.getPelvis();
         Capsule3D pelvisShapeTop = new Capsule3D(0.15, 0.1);
         pelvisShapeTop.setAxis(Axis.Y);
         pelvisShapeTop.getPosition().set(0.0, 0.0, -0.08);
         collidables.add(new KinematicsCollidable(pelvis, collisionMask, collisionGroup, pelvisShapeTop, pelvis.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));
         Capsule3D pelvisShapeBottom = new Capsule3D(0.15, 0.13);
         pelvisShapeBottom.getPosition().set(0.0, 0.0, -0.15);
         collidables.add(new KinematicsCollidable(pelvis, collisionMask, collisionGroup, pelvisShapeBottom, pelvis.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hip = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).getSuccessor();
            Sphere3D hipShape = new Sphere3D(0.12);
            hipShape.getPosition().set(0.0, 0.0, -0.07);
            collidables.add(new KinematicsCollidable(hip, collisionMask, collisionGroup, hipShape, hip.getBodyFixedFrame(), minimumSafeDistance));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         int collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         int collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()), legNames.get(RobotSide.LEFT), legNames.get(RobotSide.RIGHT));

         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         Sphere3D handShape = new Sphere3D(0.08);
         collidables.add(new KinematicsCollidable(hand, collisionMask, collisionGroup, handShape, hand.getBodyFixedFrame(), minimumSafeDistance));

         RigidBodyBasics forearm = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getSuccessor();
         Capsule3D forearmShape = new Capsule3D(0.25, 0.075);
         forearmShape.getPosition().setX(-0.02);
         forearmShape.getPosition().setY(robotSide.negateIfRightSide(0.125));
         forearmShape.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(forearm, collisionMask, collisionGroup, forearmShape, forearm.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));

//         RigidBodyBasics upperarm = fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL).getSuccessor();
//         Capsule3D upperarmShape = new Capsule3D(0.25, 0.075);
//         upperarmShape.getPosition().setY(robotSide.negateIfRightSide(0.125));
//         upperarmShape.setAxis(Axis.Y);
//         collidables.add(new KinematicsCollidable(upperarm, collisionMask, collisionGroup, upperarmShape, upperarm.getParentJoint().getFrameAfterJoint(), minimumSafeDistance));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         int collisionMask = helper.getCollisionMask(legNames.get(robotSide));
         int collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics upperLeg = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getSuccessor();
         Capsule3D upperLegShapeTop = new Capsule3D(0.25, 0.1);
         upperLegShapeTop.getPosition().set(0.02, robotSide.negateIfLeftSide(0.01), 0.08);
         RotationMatrixTools.applyPitchRotation(-Math.toRadians(8.0), upperLegShapeTop.getAxis(), upperLegShapeTop.getAxis());
         collidables.add(new KinematicsCollidable(upperLeg, collisionMask, collisionGroup, upperLegShapeTop, upperLeg.getBodyFixedFrame(), minimumSafeDistance));
         Capsule3D upperLegShapeBottom = new Capsule3D(0.20, 0.1);
         upperLegShapeBottom.getPosition().set(0.01, robotSide.negateIfLeftSide(0.01), -0.12);
         RotationMatrixTools.applyPitchRotation(Math.toRadians(12.0), upperLegShapeBottom.getAxis(), upperLegShapeBottom.getAxis());
         collidables.add(new KinematicsCollidable(upperLeg, collisionMask, collisionGroup, upperLegShapeBottom, upperLeg.getBodyFixedFrame(), minimumSafeDistance));
      }

      return collidables;
   }
}
