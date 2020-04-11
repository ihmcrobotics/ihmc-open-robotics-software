package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * Collision model for Atlas used for simulating shape-to-shape collisions. It is used only with
 * {@link ExperimentalSimulation} and not with the default SCS physics engine.
 * <p>
 * {@link ExperimentalSimulation} can be used instead of the default SCS physics engine using
 * {@link DRCSCSInitialSetup#setUseExperimentalPhysicsEngine(boolean)}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
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
      long collisionMask = helper.getCollisionMask(robotCollisionMask);
      long collisionGroup = helper.createCollisionGroup(otherCollisionMasks);

      { // Torso
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         Box3D chestCoreShape = new Box3D(0.4, 0.35, 0.5);
         chestCoreShape.getPosition().set(-0.093, 0.0, 0.28);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestCoreShape, torsoFrame));

         Box3D chestFrontLowShape = new Box3D(0.12, 0.2, 0.3);
         chestFrontLowShape.getPosition().set(0.197, 0.0, 0.158);
         chestFrontLowShape.getOrientation().setQuaternion(0.0, 0.1, 0.0, 1.0);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontLowShape, torsoFrame));

         Capsule3D chestFrontHighShape = new Capsule3D(0.3, 0.15);
         chestFrontHighShape.getPosition().set(0.147, 0.0, 0.418);
         chestFrontHighShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontHighShape, torsoFrame));

         Box3D chestTopBackShape = new Box3D(0.5, 0.35, 0.18);
         chestTopBackShape.getPosition().set(-0.043, 0.0, 0.598);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestTopBackShape, torsoFrame));

         Capsule3D chestHeadGuardShape = new Capsule3D(0.3, 0.08);
         chestHeadGuardShape.getPosition().set(0.267, 0.0, 0.623);
         chestHeadGuardShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestHeadGuardShape, torsoFrame));

         Box3D chestTopShape = new Box3D(0.3, 0.35, 0.15);
         chestTopShape.getPosition().set(0.087, 0.0, 0.758);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestTopShape, torsoFrame));
      }

      { // Pelvis
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
         ReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();

         Capsule3D pelvisShape = new Capsule3D(0.05, 0.2);
         pelvisShape.getPosition().set(0.012, 0.0, 0.037);
         pelvisShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape, pelvisFrame));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Thigh
            JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            MovingReferenceFrame hipPitchFrame = hipPitch.getFrameAfterJoint();
            RigidBodyBasics thigh = hipPitch.getSuccessor();

            Capsule3D thighTopShape = new Capsule3D(0.1, 0.09);
            thighTopShape.getPosition().set(0.0, 0.0, -0.1);
            thighTopShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape, hipPitchFrame));

            Capsule3D thighFrontShape = new Capsule3D(0.15, 0.085);
            thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
            thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape, hipPitchFrame));

            Capsule3D thighBackShape = new Capsule3D(0.15, 0.085);
            thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
            thighBackShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape, hipPitchFrame));
         }

         { // Shin
            JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            MovingReferenceFrame kneePitchFrame = kneePitch.getFrameAfterJoint();
            RigidBodyBasics shin = kneePitch.getSuccessor();

            Capsule3D shinShape = new Capsule3D(0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape, kneePitchFrame));
         }

         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
            RigidBodyBasics foot = ankleRoll.getSuccessor();

            Box3D footCoreShape = new Box3D(0.2, 0.14, 0.055);
            footCoreShape.getPosition().set(0.015, 0.0, -0.05);
            collidables.add(new Collidable(foot, collisionMask, collisionGroup, footCoreShape, ankleRollFrame));

            Box3D footFrontShape = new Box3D(0.06, 0.1, 0.055);
            footFrontShape.getPosition().set(0.145, 0.0, -0.05);
            collidables.add(new Collidable(foot, collisionMask, collisionGroup, footFrontShape, ankleRollFrame));

            for (RobotSide footSide : RobotSide.values)
            {
               PointShape3D footFrontCorner = new PointShape3D(new Point3D(0.17, footSide.negateIfRightSide(0.035), -0.077));
               collidables.add(new Collidable(foot, collisionMask, collisionGroup, footFrontCorner, ankleRollFrame));

               PointShape3D footBackCorner = new PointShape3D(new Point3D(-0.07, footSide.negateIfRightSide(0.06), -0.077));
               collidables.add(new Collidable(foot, collisionMask, collisionGroup, footBackCorner, ankleRollFrame));
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Upper arm
            JointBasics shoulderRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), multiBodySystem);
            MovingReferenceFrame shoulderRollFrame = shoulderRoll.getFrameAfterJoint();
            RigidBodyBasics upperArm = shoulderRoll.getSuccessor();

            Capsule3D upperArmShape = new Capsule3D(0.27, 0.07);
            upperArmShape.getPosition().set(-0.005, robotSide.negateIfRightSide(0.15), -0.01);
            upperArmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(upperArm, collisionMask, collisionGroup, upperArmShape, shoulderRollFrame));
         }

         { // Forearm
            JointBasics elbowJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);
            MovingReferenceFrame elbowFrame = elbowJoint.getFrameAfterJoint();
            RigidBodyBasics forearm = elbowJoint.getSuccessor();

            Capsule3D forearmShape = new Capsule3D(0.27, 0.07);
            forearmShape.getPosition().set(-0.005, robotSide.negateIfRightSide(0.15), -0.01);
            forearmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape, elbowFrame));
         }

         if (atlasRobotVersion.hasRobotiqHands())
         { // RobotiQ hand
            JointBasics lastWristJoint = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), multiBodySystem);
            MovingReferenceFrame wristFrame = lastWristJoint.getFrameAfterJoint();
            RigidBodyBasics hand = lastWristJoint.getSuccessor();

            Box3D footFrontShape = new Box3D(0.125, 0.1, 0.125);
            footFrontShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.17), 0.0);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, footFrontShape, wristFrame));
         }
      }

      return collidables;
   }
}
