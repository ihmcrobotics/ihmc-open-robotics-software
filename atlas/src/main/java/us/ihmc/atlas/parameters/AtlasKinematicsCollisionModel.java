package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.shapes.FrameSTPBox3D;
import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class AtlasKinematicsCollisionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;

   public AtlasKinematicsCollisionModel(HumanoidJointNameMap jointMap)
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
         FrameCapsule3D headShapeMultisense = new FrameCapsule3D(headFrame, 0.08, 0.115);
         headShapeMultisense.getPosition().set(0.03, 0.0, 0.03);
         headShapeMultisense.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShapeMultisense));

         // Torso ---------------------------------------------------------------------
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();

         MovingReferenceFrame beforeNeckFrame = head.getParentJoint().getFrameBeforeJoint();
         // Cover the head guards that are part of the torso actually.
         FrameCapsule3D headguardShapeFront = new FrameCapsule3D(beforeNeckFrame, 0.25, 0.11);
         headguardShapeFront.getPosition().set(0.0, 0.0, 0.0);
         headguardShapeFront.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, headguardShapeFront));

         // Sideway capsule covering the shoulders and "pecs"-part
         FrameCapsule3D torsoShapeFrontShoulder = new FrameCapsule3D(torsoFrame, 0.2, 0.2);
         torsoShapeFrontShoulder.getPosition().set(0.14, 0.0, 0.415);
         torsoShapeFrontShoulder.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeFrontShoulder));
         // Capsule along the forward axis covering the top part of the "abdomen".
         FrameCapsule3D torsoShapeCenter = new FrameCapsule3D(torsoFrame, 0.25, 0.2);
         torsoShapeCenter.getPosition().set(-0.015, 0.0, 0.265);
         torsoShapeCenter.getAxis().set(Axis3D.X);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeCenter));
         // Capsule along the forward axis covering the bottom part of the "abdomen" and of the chest.
         FrameCapsule3D torsoShapeBottomCenter = new FrameCapsule3D(torsoFrame, 0.25, 0.2);
         torsoShapeBottomCenter.getPosition().set(-0.045, 0.0, 0.115);
         torsoShapeBottomCenter.getAxis().set(Axis3D.X);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomCenter));
         // Capsule along the forward axis covering the bottom left corner of the chest.
         for (RobotSide robotSide : RobotSide.values)
         {
            FrameCapsule3D torsoShapeBottomSideCorner = new FrameCapsule3D(torsoFrame, 0.25, 0.1);
            torsoShapeBottomSideCorner.getPosition().set(-0.115, robotSide.negateIfRightSide(0.1), 0.067);
            torsoShapeBottomSideCorner.getAxis().set(Axis3D.X);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomSideCorner));
         }

         // Pelvis ---------------------------------------------------------------------
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D pelvisShape = new FrameCapsule3D(pelvisFrame, 0.05, 0.22);
         pelvisShape.getAxis().set(Axis3D.Z);
         pelvisShape.getPosition().set(0.012, 0.0, 0.037);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape));

         // Legs ---------------------------------------------------------------------
         for (RobotSide robotSide : RobotSide.values)
         {
            JointBasics hipPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            RigidBodyBasics thigh = hipPitchJoint.getSuccessor();
            ReferenceFrame thighFrame = hipPitchJoint.getFrameAfterJoint();

            FrameCapsule3D thighShapeTop = new FrameCapsule3D(thighFrame, 0.1, 0.09);
            thighShapeTop.getPosition().set(0.0, 0.0, -0.1);
            thighShapeTop.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeTop));

            FrameCapsule3D thighShapeBottom = new FrameCapsule3D(thighFrame, 0.15, 0.085);
            thighShapeBottom.getPosition().set(-0.018, 0.0, -0.25);
            thighShapeBottom.getAxis().set(0.22, 0.0, 1.0);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBottom));

            FrameCapsule3D thighShapeBack = new FrameCapsule3D(thighFrame, 0.15, 0.085);
            thighShapeBack.getPosition().set(-0.05, 0.0, -0.15);
            thighShapeBack.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBack));

            JointBasics shinPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            RigidBodyBasics shin = shinPitchJoint.getSuccessor();
            ReferenceFrame shinFrame = shinPitchJoint.getFrameAfterJoint();

            FrameCapsule3D shinShape = new FrameCapsule3D(shinFrame, 0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(0.1, 0.0, 1.0);
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         long collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()));

         RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
         ReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D handShapeKnob = new FrameCapsule3D(handFrame, 0.07, 0.06);
         handShapeKnob.getPosition().set(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         handShapeKnob.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeKnob));

         JointBasics elbowJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);
         RigidBodyBasics forearm = elbowJoint.getSuccessor();
         ReferenceFrame elbowFrame = elbowJoint.getFrameAfterJoint();
         FrameCapsule3D forearmShape = new FrameCapsule3D(elbowFrame, 0.31, 0.1);
         forearmShape.getPosition().set(-0.01, robotSide.negateIfRightSide(0.12), -0.01);
         forearmShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape));
      }

      collidables.addAll(setupLegCollisions(multiBodySystem, helper));
      
      return collidables;
   }

   private List<Collidable> setupLegCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String leftFootName = "LeftFoot";
      String rightFootName = "RightFoot";

      // Collision between feet
      { // Left foot
         long collisionMask = helper.getCollisionMask(leftFootName);
         long collisionGroup = helper.createCollisionGroup(rightFootName);

         JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_ROLL), multiBodySystem);
         MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
         RigidBodyBasics foot = ankleRoll.getSuccessor();

         // Using a STP box so the sole is slightly rounded allowing for continuous and smooth contact with the ground.
         FrameSTPBox3D footShape = new FrameSTPBox3D(ankleRollFrame, 0.26, 0.14, 0.055);
         footShape.getPosition().set(0.045, 0.0, -0.05);
         footShape.setMargins(1.0e-5, 4.0e-4);
         collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
      }

      { // Right foot
         long collisionMask = helper.getCollisionMask(rightFootName);
         long collisionGroup = helper.createCollisionGroup(leftFootName);

         JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_ROLL), multiBodySystem);
         MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
         RigidBodyBasics foot = ankleRoll.getSuccessor();

         // Using a STP box so the sole is slightly rounded allowing for continuous and smooth contact with the ground.
         FrameSTPBox3D footShape = new FrameSTPBox3D(ankleRollFrame, 0.26, 0.14, 0.055);
         footShape.getPosition().set(0.045, 0.0, -0.05);
         footShape.setMargins(1.0e-5, 4.0e-4);
         collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
      }

      // Collision between thighs
      String leftThighName = "LeftThigh";
      String rightThighName = "RightThigh";

      { // Left thigh
         long collisionMask = helper.getCollisionMask(leftThighName);
         long collisionGroup = helper.createCollisionGroup(rightThighName);

         JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH), multiBodySystem);
         MovingReferenceFrame hipPitchFrame = hipPitch.getFrameAfterJoint();
         RigidBodyBasics thigh = hipPitch.getSuccessor();

         FrameCapsule3D thighTopShape = new FrameCapsule3D(hipPitchFrame, 0.1, 0.09);
         thighTopShape.getPosition().set(0.0, 0.0, -0.1);
         thighTopShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape));

         FrameCapsule3D thighFrontShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
         thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape));

         FrameCapsule3D thighBackShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
         thighBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape));
      }

      { // Right thigh
         long collisionMask = helper.getCollisionMask(rightThighName);
         long collisionGroup = helper.createCollisionGroup(leftThighName);

         JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH), multiBodySystem);
         MovingReferenceFrame hipPitchFrame = hipPitch.getFrameAfterJoint();
         RigidBodyBasics thigh = hipPitch.getSuccessor();

         FrameCapsule3D thighTopShape = new FrameCapsule3D(hipPitchFrame, 0.1, 0.09);
         thighTopShape.getPosition().set(0.0, 0.0, -0.1);
         thighTopShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape));

         FrameCapsule3D thighFrontShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
         thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape));

         FrameCapsule3D thighBackShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
         thighBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape));
      }

      // Collision between shins
      String leftShinName = "LeftShin";
      String rightShinName = "RightShin";

      { // Left Shin
         long collisionMask = helper.getCollisionMask(leftShinName);
         long collisionGroup = helper.createCollisionGroup(rightShinName);

         JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH), multiBodySystem);
         MovingReferenceFrame kneePitchFrame = kneePitch.getFrameAfterJoint();
         RigidBodyBasics shin = kneePitch.getSuccessor();

         FrameCapsule3D shinShape = new FrameCapsule3D(kneePitchFrame, 0.3, 0.08);
         shinShape.getPosition().set(0.015, 0.0, -0.2);
         shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
         collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
      }

      { // Right Shin
         long collisionMask = helper.getCollisionMask(rightThighName);
         long collisionGroup = helper.createCollisionGroup(leftThighName);
         RobotSide robotSide = RobotSide.RIGHT;

         JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH), multiBodySystem);
         MovingReferenceFrame kneePitchFrame = kneePitch.getFrameAfterJoint();
         RigidBodyBasics shin = kneePitch.getSuccessor();

         FrameCapsule3D shinShape = new FrameCapsule3D(kneePitchFrame, 0.3, 0.08);
         shinShape.getPosition().set(0.015, 0.0, -0.2);
         shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
         collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
      }

      return collidables;
   }
}
