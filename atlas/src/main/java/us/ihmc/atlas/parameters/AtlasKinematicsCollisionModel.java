package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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

public class AtlasKinematicsCollisionModel implements RobotCollisionModel
{
   private final DRCRobotJointMap jointMap;

   public AtlasKinematicsCollisionModel(DRCRobotJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   // TODO Need to implement the RobotiQ hands, this implementation only cover the knobs.
   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      CollidableHelper helper = new CollidableHelper();
      List<Collidable> collidables = new ArrayList<>();

      /*
       * The following is only meant for the arms (forearm and hand) collision vs body (head, chest,
       * pelvis, and legs) and collision between the 2 arms. For other types of collisions, it's probably
       * better to implement them as separate groups and avoid interaction with this set of collidables.
       */
      String bodyName = "Body"; // head + chest + pelvis + legs
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);

         // Head ---------------------------------------------------------------------
         MovingReferenceFrame headFrame = head.getBodyFixedFrame();
         // Covers the whole multisense.
         Capsule3D headShapeMultisense = new Capsule3D(0.08, 0.115);
         headShapeMultisense.getPosition().set(0.03, 0.0, 0.03);
         headShapeMultisense.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShapeMultisense, headFrame));

         // Torso ---------------------------------------------------------------------
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();

         MovingReferenceFrame beforeNeckFrame = head.getParentJoint().getFrameBeforeJoint();
         // Cover the head guards that are part of the torso actually.
         Capsule3D headguardShapeFront = new Capsule3D(0.25, 0.11);
         headguardShapeFront.getPosition().set(0.0, 0.0, 0.0);
         headguardShapeFront.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, headguardShapeFront, beforeNeckFrame));

         // Sideway capsule covering the shoulders and "pecs"-part
         Capsule3D torsoShapeFrontShoulder = new Capsule3D(0.2, 0.2);
         torsoShapeFrontShoulder.getPosition().set(0.14, 0.0, 0.415);
         torsoShapeFrontShoulder.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeFrontShoulder, torsoFrame));
         // Capsule along the forward axis covering the top part of the "abdomen".
         Capsule3D torsoShapeCenter = new Capsule3D(0.25, 0.2);
         torsoShapeCenter.getPosition().set(-0.015, 0.0, 0.265);
         torsoShapeCenter.getAxis().set(Axis3D.X);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeCenter, torsoFrame));
         // Capsule along the forward axis covering the bottom part of the "abdomen" and of the chest.
         Capsule3D torsoShapeBottomCenter = new Capsule3D(0.25, 0.2);
         torsoShapeBottomCenter.getPosition().set(-0.045, 0.0, 0.115);
         torsoShapeBottomCenter.getAxis().set(Axis3D.X);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomCenter, torsoFrame));
         // Capsule along the forward axis covering the bottom left corner of the chest.
         for (RobotSide robotSide : RobotSide.values)
         {
            Capsule3D torsoShapeBottomSideCorner = new Capsule3D(0.25, 0.1);
            torsoShapeBottomSideCorner.getPosition().set(-0.115, robotSide.negateIfRightSide(0.1), 0.067);
            torsoShapeBottomSideCorner.getAxis().set(Axis3D.X);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomSideCorner, torsoFrame));
         }

         // Pelvis ---------------------------------------------------------------------
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         Capsule3D pelvisShape = new Capsule3D(0.05, 0.22);
         pelvisShape.getAxis().set(Axis3D.Z);
         pelvisShape.getPosition().set(0.012, 0.0, 0.037);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape, pelvisFrame));

         // Legs ---------------------------------------------------------------------
         for (RobotSide robotSide : RobotSide.values)
         {
            JointBasics hipPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            RigidBodyBasics thigh = hipPitchJoint.getSuccessor();
            ReferenceFrame thighFrame = hipPitchJoint.getFrameAfterJoint();

            Capsule3D thighShapeTop = new Capsule3D(0.1, 0.09);
            thighShapeTop.getPosition().set(0.0, 0.0, -0.1);
            thighShapeTop.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeTop, thighFrame));

            Capsule3D thighShapeBottom = new Capsule3D(0.15, 0.085);
            thighShapeBottom.getPosition().set(-0.018, 0.0, -0.25);
            thighShapeBottom.getAxis().set(0.22, 0.0, 1.0);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBottom, thighFrame));

            Capsule3D thighShapeBack = new Capsule3D(0.15, 0.085);
            thighShapeBack.getPosition().set(-0.05, 0.0, -0.15);
            thighShapeBack.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBack, thighFrame));

            JointBasics shinPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            RigidBodyBasics shin = shinPitchJoint.getSuccessor();
            ReferenceFrame shinFrame = shinPitchJoint.getFrameAfterJoint();

            Capsule3D shinShape = new Capsule3D(0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(0.1, 0.0, 1.0);
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape, shinFrame));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         long collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()));

         RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
         ReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
         Capsule3D handShapeKnob = new Capsule3D(0.07, 0.06);
         handShapeKnob.getPosition().set(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         handShapeKnob.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeKnob, handFrame));

         JointBasics elbowJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);
         RigidBodyBasics forearm = elbowJoint.getSuccessor();
         ReferenceFrame elbowFrame = elbowJoint.getFrameAfterJoint();
         Capsule3D forearmShape = new Capsule3D(0.31, 0.1);
         forearmShape.getPosition().set(-0.01, robotSide.negateIfRightSide(0.12), -0.01);
         forearmShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape, elbowFrame));
      }

      return collidables;
   }
}
