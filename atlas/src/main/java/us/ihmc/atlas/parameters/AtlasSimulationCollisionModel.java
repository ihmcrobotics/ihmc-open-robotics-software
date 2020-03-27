package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSimulationCollisionModel implements RobotCollisionModel
{
   private final DRCRobotJointMap jointMap;

   private CollidableHelper helper;
   private String[] otherCollisionMasks;
   private String robotCollisionMask;

   private AtlasRobotVersion atlasRobotVersion;

   public AtlasSimulationCollisionModel(DRCRobotJointMap jointMap, AtlasRobotVersion atlasRobotVersion)
   {
      this.jointMap = jointMap;
      this.atlasRobotVersion = atlasRobotVersion;
   }

   public void setCollidableHelper(CollidableHelper helper, String robotCollisionMask, String... otherCollisionMasks)
   {
      this.helper = helper;
      this.robotCollisionMask = robotCollisionMask;
      this.otherCollisionMasks = otherCollisionMasks;
   }

   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      List<Collidable> collidables = new ArrayList<>();
      int collisionMask = helper.getCollisionMask(robotCollisionMask);
      int collisionGroup = helper.createCollisionGroup(otherCollisionMasks);

      { // Torso
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         Box3D chestCoreShape = new Box3D(0.4, 0.35, 0.5);
         chestCoreShape.getPosition().set(-0.06-0.05, 0.0, -0.065-0.03);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestCoreShape, torso.getBodyFixedFrame()));

         Box3D chestFrontLowShape = new Box3D(0.12, 0.2, 0.3);
         chestFrontLowShape.getPosition().set(0.23-0.05, 0.0, -0.16-0.03);
         chestFrontLowShape.getOrientation().setQuaternion(0.0, 0.1, 0.0, 1.0);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontLowShape, torso.getBodyFixedFrame()));

         Capsule3D chestFrontHighShape = new Capsule3D(0.3, 0.15);
         chestFrontHighShape.getPosition().set(0.18-0.05, 0.0, 0.1-0.03);
         chestFrontHighShape.getAxis().set(Axis.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontHighShape, torso.getBodyFixedFrame()));

         Box3D chestTopBackShape = new Box3D(0.5, 0.35, 0.18);
         chestTopBackShape.getPosition().set(-0.01-0.05, 0.0, 0.28-0.03);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestTopBackShape, torso.getBodyFixedFrame()));

         Capsule3D chestHeadGuardShape = new Capsule3D(0.3, 0.08);
         chestHeadGuardShape.getPosition().set(0.3-0.05, 0.0, 0.305-0.03);
         chestHeadGuardShape.getAxis().set(Axis.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestHeadGuardShape, torso.getBodyFixedFrame()));

         Box3D chestTopShape = new Box3D(0.3, 0.35, 0.15);
         chestTopShape.getPosition().set(0.12-0.05, 0.0, 0.44-0.03);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestTopShape, torso.getBodyFixedFrame()));
      }

      { // Pelvis
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);

         Capsule3D pelvisShape = new Capsule3D(0.05, 0.2);
         pelvisShape.getPosition().set(0.0, 0.0, 0.01);
         pelvisShape.getAxis().set(Axis.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape, pelvis.getBodyFixedFrame()));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Thigh
            JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            RigidBodyBasics thigh = hipPitch.getSuccessor();

            Capsule3D thighTopShape = new Capsule3D(0.1, 0.09);
            thighTopShape.getPosition().set(0.0, 0.0, -0.1);
            thighTopShape.getAxis().set(Axis.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape, hipPitch.getFrameAfterJoint()));

            Capsule3D thighFrontShape = new Capsule3D(0.15, 0.085);
            thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
            thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape, hipPitch.getFrameAfterJoint()));

            Capsule3D thighBackShape = new Capsule3D(0.15, 0.085);
            thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
            thighBackShape.getAxis().set(Axis.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape, hipPitch.getFrameAfterJoint()));
         }

         { // Shin
            JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            RigidBodyBasics shin = kneePitch.getSuccessor();

            Capsule3D shinShape = new Capsule3D(0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape, kneePitch.getFrameAfterJoint()));
         }

         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            RigidBodyBasics foot = ankleRoll.getSuccessor();

            Box3D footCoreShape = new Box3D(0.2, 0.14, 0.055);
            footCoreShape.getPosition().set(0.015, 0.0, -0.05);
            collidables.add(new Collidable(foot, collisionMask, collisionGroup, footCoreShape, ankleRoll.getFrameAfterJoint()));

            Box3D footFrontShape = new Box3D(0.06, 0.1, 0.055);
            footFrontShape.getPosition().set(0.145, 0.0, -0.05);
            collidables.add(new Collidable(foot, collisionMask, collisionGroup, footFrontShape, ankleRoll.getFrameAfterJoint()));

            for (RobotSide footSide : RobotSide.values)
            {
               PointShape3D footFrontCorner = new PointShape3D(new Point3D(0.17, footSide.negateIfRightSide(0.035), -0.077));
               collidables.add(new Collidable(foot, collisionMask, collisionGroup, footFrontCorner, ankleRoll.getFrameAfterJoint()));

               PointShape3D footBackCorner = new PointShape3D(new Point3D(-0.07, footSide.negateIfRightSide(0.06), -0.077));
               collidables.add(new Collidable(foot, collisionMask, collisionGroup, footBackCorner, ankleRoll.getFrameAfterJoint()));
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Upper arm
            JointBasics shoulderRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), multiBodySystem);
            RigidBodyBasics upperArm = shoulderRoll.getSuccessor();

            Capsule3D upperArmShape = new Capsule3D(0.27, 0.07);
            upperArmShape.getPosition().set(-0.005, robotSide.negateIfRightSide(0.15), -0.01);
            upperArmShape.getAxis().set(Axis.Y);
            collidables.add(new Collidable(upperArm, collisionMask, collisionGroup, upperArmShape, shoulderRoll.getFrameAfterJoint()));
         }

         { // Forearm
            JointBasics elbowJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);
            RigidBodyBasics forearm = elbowJoint.getSuccessor();

            Capsule3D forearmShape = new Capsule3D(0.27, 0.07);
            forearmShape.getPosition().set(-0.005, robotSide.negateIfRightSide(0.15), -0.01);
            forearmShape.getAxis().set(Axis.Y);
            collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape, elbowJoint.getFrameAfterJoint()));
         }

         if (atlasRobotVersion.hasRobotiqHands())
         { // RobotiQ hand
            JointBasics lastWristJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), multiBodySystem);
            RigidBodyBasics hand = lastWristJoint.getSuccessor();

            Box3D footFrontShape = new Box3D(0.125, 0.1, 0.125);
            footFrontShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.17), 0.0);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, footFrontShape, lastWristJoint.getFrameAfterJoint()));
         }
      }

      return collidables;
   }
}
