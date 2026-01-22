package us.ihmc.zulu.parameters.model;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHelper;

import java.util.ArrayList;
import java.util.List;

public class ZuluKSTKinematicsCollisionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;

   public ZuluKSTKinematicsCollisionModel(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

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
         if (head != null)
         {
            MovingReferenceFrame headFrame = head.getBodyFixedFrame();
            FrameCapsule3D headShapeMultisense = new FrameCapsule3D(headFrame, 0.08, 0.15);
            headShapeMultisense.getPosition().set(0.03, 0.0, 0.03);
            headShapeMultisense.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(head, collisionMask, collisionGroup, headShapeMultisense));
         }

         // Torso ---------------------------------------------------------------------
         if (torso != null)
         {
            MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
            // Capsule along the forward axis covering the bottom part of the "abdomen" and of the chest.
            FrameCapsule3D torsoShapeBottomCenter = new FrameCapsule3D(torsoFrame, 0.15, 0.17);
            torsoShapeBottomCenter.getPosition().set(-0.01, 0.0, 0.22);
            torsoShapeBottomCenter.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomCenter));
         }

         // Pelvis ---------------------------------------------------------------------
         if (pelvis != null)
         {
            MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D pelvisShape = new FrameCapsule3D(pelvisFrame, 0.03, 0.135);
            pelvisShape.getAxis().set(Axis3D.Z);
            pelvisShape.getPosition().set(-0.06, 0.0, -0.02);
            collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape));
         }

         // Legs ---------------------------------------------------------------------
         for (RobotSide robotSide : RobotSide.values)
         {
            JointBasics hipYawJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), multiBodySystem);
            RigidBodyBasics hip = hipYawJoint.getSuccessor();
            ReferenceFrame hipFrame = hipYawJoint.getFrameAfterJoint();
            FrameCapsule3D hipShape = new FrameCapsule3D(hipFrame, 0.07, 0.1);
            hipShape.getPosition().set(-0.01, robotSide.negateIfLeftSide(0.045), 0.035);
            hipShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(hip, collisionMask, collisionGroup, hipShape));

            JointBasics hipPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            RigidBodyBasics thigh = hipPitchJoint.getSuccessor();
            ReferenceFrame thighFrame = hipPitchJoint.getFrameAfterJoint();

            FrameCapsule3D thighShapeTop = new FrameCapsule3D(thighFrame, 0.085, 0.09);
            thighShapeTop.getPosition().set(0.0, robotSide.negateIfRightSide(0.04), -0.05);
            thighShapeTop.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeTop));

            FrameCapsule3D thighShapeMid = new FrameCapsule3D(thighFrame, 0.085, 0.11);
            thighShapeMid.getPosition().set(0.03, robotSide.negateIfRightSide(0.03), -0.12);
            thighShapeMid.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeMid));

            FrameCapsule3D thighShapeBottom = new FrameCapsule3D(thighFrame, 0.085, 0.09);
            thighShapeBottom.getPosition().set(0.05, robotSide.negateIfRightSide(0.02), -0.21);
            thighShapeBottom.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBottom));

            JointBasics shinPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            RigidBodyBasics shin = shinPitchJoint.getSuccessor();
            ReferenceFrame shinFrame = shinPitchJoint.getFrameAfterJoint();

            FrameCapsule3D shinShape = new FrameCapsule3D(shinFrame, 0.3, 0.1);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(0.15, 0.0, 1.0);
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         long collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(bodyName, armNames.get(robotSide.getOppositeSide()));

         JointBasics shoulderJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), multiBodySystem);
         if (shoulderJoint != null)
         {
            RigidBodyBasics upperArm = shoulderJoint.getSuccessor();
            ReferenceFrame shoulderFrame = shoulderJoint.getFrameAfterJoint();
            FrameCapsule3D upperArmShape = new FrameCapsule3D(shoulderFrame, 0.20, 0.06);
            upperArmShape.getPosition().set(0.01, 0.0, -0.16);
            upperArmShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(upperArm, collisionMask, collisionGroup, upperArmShape));
         }

         JointBasics elbowJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
         if (elbowJoint != null)
         {
            RigidBodyBasics forearm = elbowJoint.getSuccessor();
            ReferenceFrame elbowFrame = elbowJoint.getFrameAfterJoint();
            FrameCapsule3D forearmShape = new FrameCapsule3D(elbowFrame, 0.2, 0.065);
            forearmShape.getPosition().set(-0.02, 0.0, -0.1);
            forearmShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape));
         }

         RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
         if (hand != null)
         {
            ReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D handShapeKnob = new FrameCapsule3D(handFrame, 0.06, 0.06);
            handShapeKnob.getPosition().set(0.0, 0.0, 0.0);
            handShapeKnob.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeKnob));
         }
      }

      return collidables;
   }
}
