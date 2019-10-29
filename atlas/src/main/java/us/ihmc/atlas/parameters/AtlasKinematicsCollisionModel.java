package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidableHelper;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AtlasKinematicsCollisionModel implements HumanoidRobotKinematicsCollisionModel
{
   public AtlasKinematicsCollisionModel()
   {
   }

   // TODO Need to implement the RobotiQ hands, this implementation only cover the knobs.
   @Override
   public List<KinematicsCollidable> getRobotCollidables(FullHumanoidRobotModel fullRobotModel)
   {
      KinematicsCollidableHelper helper = new KinematicsCollidableHelper();
      List<KinematicsCollidable> collidables = new ArrayList<>();

      double minimumSafeDistance = 0.0;

      /*
       * The following is only meant for the arms (forearm and hand) collision vs body (head, chest,
       * pelvis, and legs) and collision between the 2 arms. For other types of collisions, it's probably
       * better to implement them as separate groups and avoid interaction with this set of collidables.
       */
      String bodyName = "Body"; // head + chest + pelvis + legs
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");

      { // Body
         int collisionMask = helper.getCollisionMask(bodyName);
         int collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics head = fullRobotModel.getHead();
         RigidBodyBasics torso = fullRobotModel.getChest();
         RigidBodyBasics pelvis = fullRobotModel.getPelvis();

         // Head ---------------------------------------------------------------------
         MovingReferenceFrame headFrame = head.getBodyFixedFrame();
         // Covers the whole multisense.
         Capsule3D headShapeMultisense = new Capsule3D(0.08, 0.115);
         headShapeMultisense.getPosition().set(0.03, 0.0, 0.03);
         headShapeMultisense.setAxis(Axis.Z);
         collidables.add(new KinematicsCollidable(head, collisionMask, collisionGroup, headShapeMultisense, headFrame, minimumSafeDistance));

         // Torso ---------------------------------------------------------------------
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();

         MovingReferenceFrame beforeNeckFrame = head.getParentJoint().getFrameBeforeJoint();
         // Cover the head guards that are part of the torso actually.
         Capsule3D headguardShapeFront = new Capsule3D(0.25, 0.11);
         headguardShapeFront.getPosition().set(0.0, 0.0, 0.0);
         headguardShapeFront.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, headguardShapeFront, beforeNeckFrame, minimumSafeDistance));

         // Sideway capsule covering the shoulders and "pecs"-part
         Capsule3D torsoShapeFrontShoulder = new Capsule3D(0.2, 0.2);
         torsoShapeFrontShoulder.getPosition().set(0.14, 0.0, 0.415);
         torsoShapeFrontShoulder.getAxis().set(Axis.Y);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeFrontShoulder, torsoFrame, minimumSafeDistance));
         // Capsule along the forward axis covering the top part of the "abdomen".
         Capsule3D torsoShapeCenter = new Capsule3D(0.25, 0.2);
         torsoShapeCenter.getPosition().set(-0.015, 0.0, 0.265);
         torsoShapeCenter.setAxis(Axis.X);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeCenter, torsoFrame, minimumSafeDistance));
         // Capsule along the forward axis covering the bottom part of the "abdomen" and of the chest.
         Capsule3D torsoShapeBottomCenter = new Capsule3D(0.25, 0.2);
         torsoShapeBottomCenter.getPosition().set(-0.045, 0.0, 0.115);
         torsoShapeBottomCenter.setAxis(Axis.X);
         collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeBottomCenter, torsoFrame, minimumSafeDistance));
         // Capsule along the forward axis covering the bottom left corner of the chest.
         for (RobotSide robotSide : RobotSide.values)
         {
            Capsule3D torsoShapeBottomSideCorner = new Capsule3D(0.25, 0.1);
            torsoShapeBottomSideCorner.getPosition().set(-0.115, robotSide.negateIfRightSide(0.1), 0.067);
            torsoShapeBottomSideCorner.setAxis(Axis.X);
            collidables.add(new KinematicsCollidable(torso, collisionMask, collisionGroup, torsoShapeBottomSideCorner, torsoFrame, minimumSafeDistance));
         }

         // Pelvis ---------------------------------------------------------------------
         MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
         Capsule3D pelvisShape = new Capsule3D(0.05, 0.22);
         pelvisShape.setAxis(Axis.Z);
         pelvisShape.getPosition().set(0.0, 0.0, 0.01);
         collidables.add(new KinematicsCollidable(pelvis, collisionMask, collisionGroup, pelvisShape, pelvisFrame, minimumSafeDistance));

         // Legs ---------------------------------------------------------------------
         for (RobotSide robotSide : RobotSide.values)
         {
            OneDoFJointBasics hipPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH);
            RigidBodyBasics thigh = hipPitchJoint.getSuccessor();
            ReferenceFrame thighFrame = hipPitchJoint.getFrameAfterJoint();

            Capsule3D thighShapeTop = new Capsule3D(0.1, 0.09);
            thighShapeTop.getPosition().set(0.0, 0.0, -0.1);
            thighShapeTop.setAxis(Axis.Z);
            collidables.add(new KinematicsCollidable(thigh, collisionMask, collisionGroup, thighShapeTop, thighFrame, minimumSafeDistance));

            Capsule3D thighShapeBottom = new Capsule3D(0.15, 0.085);
            thighShapeBottom.getPosition().set(-0.018, 0.0, -0.25);
            thighShapeBottom.setAxis(new Vector3D(0.22, 0.0, 1.0));
            collidables.add(new KinematicsCollidable(thigh, collisionMask, collisionGroup, thighShapeBottom, thighFrame, minimumSafeDistance));

            Capsule3D thighShapeBack = new Capsule3D(0.15, 0.085);
            thighShapeBack.getPosition().set(-0.05, 0.0, -0.15);
            thighShapeBack.setAxis(Axis.Z);
            collidables.add(new KinematicsCollidable(thigh, collisionMask, collisionGroup, thighShapeBack, thighFrame, minimumSafeDistance));

            OneDoFJointBasics shinPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
            RigidBodyBasics shin = shinPitchJoint.getSuccessor();
            ReferenceFrame shinFrame = shinPitchJoint.getFrameAfterJoint();

            Capsule3D shinShape = new Capsule3D(0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.setAxis(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new KinematicsCollidable(shin, collisionMask, collisionGroup, shinShape, shinFrame, minimumSafeDistance));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         int collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         int collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()));

         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
         Capsule3D handShapeKnob = new Capsule3D(0.07, 0.06);
         handShapeKnob.getPosition().set(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         handShapeKnob.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(hand, collisionMask, collisionGroup, handShapeKnob, handFrame, minimumSafeDistance));

         OneDoFJointBasics elbowJoint = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL);
         RigidBodyBasics forearm = elbowJoint.getSuccessor();
         ReferenceFrame elbowFrame = elbowJoint.getFrameAfterJoint();
         Capsule3D forearmShape = new Capsule3D(0.31, 0.1);
         forearmShape.getPosition().set(-0.01, robotSide.negateIfRightSide(0.12), -0.01);
         forearmShape.setAxis(Axis.Y);
         collidables.add(new KinematicsCollidable(forearm, collisionMask, collisionGroup, forearmShape, elbowFrame, minimumSafeDistance));
      }

      return collidables;
   }
}
