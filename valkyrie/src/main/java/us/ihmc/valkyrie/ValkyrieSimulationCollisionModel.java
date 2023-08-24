package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.shapes.FrameSTPBox3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

/**
 * Collision model for Valkyrie used for simulating shape-to-shape collisions. It is used only with
 * {@link ExperimentalSimulation} and not with the default SCS physics engine.
 * <p>
 * {@link ExperimentalSimulation} can be used instead of the default SCS physics engine using
 * {@link DRCSCSInitialSetup#setUseExperimentalPhysicsEngine(boolean)}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class ValkyrieSimulationCollisionModel implements RobotCollisionModel
{
   private CollidableHelper helper;
   private String[] otherCollisionMasks;
   private final HumanoidJointNameMap jointMap;
   private String robotCollisionMask;
   private final boolean createForSimulation;

   public ValkyrieSimulationCollisionModel(HumanoidJointNameMap jointMap)
   {
      this(jointMap, true);
   }

   public ValkyrieSimulationCollisionModel(HumanoidJointNameMap jointMap, boolean createForSimulation)
   {
      this.jointMap = jointMap;
      this.createForSimulation = createForSimulation;
      // Setting default info.
      setCollidableHelper(new CollidableHelper(), jointMap.getModelName(), "other");
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
      double modelScale = jointMap.getModelScale();

      { // Head
         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         MovingReferenceFrame headFrame = head.getParentJoint().getFrameAfterJoint();
         FrameSphere3D headShape = new FrameSphere3D(headFrame, 0.15 * modelScale);
         headShape.getPosition().set(0.077, 0.0, 0.001);
         headShape.getPosition().scale(modelScale);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShape));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         { // Shoulder
            JointBasics shoulderRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), multiBodySystem);
            MovingReferenceFrame shoulderRollFrame = shoulderRoll.getFrameAfterJoint();
            FrameCapsule3D shoulderShape = new FrameCapsule3D(shoulderRollFrame, 0.10 * modelScale, 0.075 * modelScale);
            shoulderShape.getPosition().set(-0.0025, 0.0, 0.0);
            shoulderShape.getPosition().scale(modelScale);
            shoulderShape.getAxis().set(Axis3D.X);
            collidables.add(new Collidable(shoulderRoll.getPredecessor(), collisionMask, collisionGroup, shoulderShape));
         }

         { // Upper-arm
            JointBasics shoulderYaw = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), multiBodySystem);
            MovingReferenceFrame shoulderYawFrame = shoulderYaw.getFrameAfterJoint();
            FrameCapsule3D upperArmShape = new FrameCapsule3D(shoulderYawFrame, 0.25 * modelScale, 0.075 * modelScale);
            upperArmShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.14), 0.0);
            upperArmShape.getPosition().scale(modelScale);
            upperArmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(shoulderYaw.getPredecessor(), collisionMask, collisionGroup, upperArmShape));
         }

         { // Elbow
            JointBasics elbow = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
            MovingReferenceFrame elbowFrame = elbow.getFrameAfterJoint();
            FrameCapsule3D elbowShape = new FrameCapsule3D(elbowFrame, 0.08 * modelScale, 0.05 * modelScale);
            elbowShape.getPosition().set(0.0, 0.0, 0.0);
            elbowShape.getPosition().scale(modelScale);
            elbowShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(elbow.getPredecessor(), collisionMask, collisionGroup, elbowShape));
         }

         { // Forearm
            JointBasics elbow = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
            MovingReferenceFrame elbowFrame = elbow.getFrameAfterJoint();
            FrameCapsule3D forearmShape = new FrameCapsule3D(elbowFrame, 0.15 * modelScale, 0.075 * modelScale);
            forearmShape.getPosition().set(-0.03, robotSide.negateIfRightSide(0.14), 0.0);
            forearmShape.getPosition().scale(modelScale);
            forearmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(elbow.getSuccessor(), collisionMask, collisionGroup, forearmShape));
         }

         { // Hand
            RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
            MovingReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D handShape = new FrameCapsule3D(handFrame, 0.02 * modelScale, 0.055 * modelScale);
            handShape.getPosition().set(-0.007, robotSide.negateIfRightSide(0.062), -0.01);
            handShape.getPosition().scale(modelScale);
            handShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShape));
         }
      }

      { // Torso
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D chestFrontShape = new FrameCapsule3D(torsoFrame, 0.13 * modelScale, 0.12 * modelScale);
         chestFrontShape.getPosition().set(0.034, 0.0, 0.253);
         chestFrontShape.getPosition().scale(modelScale);
         chestFrontShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontShape));

         Vector3D checkBackShape = new Vector3D(0.35, 0.35, 0.4);
         if (createForSimulation)
         {
            FrameSTPBox3D chestBackShape = new FrameSTPBox3D(torsoFrame, checkBackShape);
            chestBackShape.getSize().scale(modelScale);
            chestBackShape.setMargins(1.0e-5, 4.0e-4);
            chestBackShape.getPosition().set(-0.111, 0.0, 0.208);
            chestBackShape.getPosition().scale(modelScale);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestBackShape));
         }
         else
         {
            FrameBox3D chestBackShape = new FrameBox3D(torsoFrame, checkBackShape);
            chestBackShape.getSize().scale(modelScale);
            chestBackShape.getPosition().set(-0.111, 0.0, 0.208);
            chestBackShape.getPosition().scale(modelScale);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestBackShape));
         }
      }

      { // Pelvis
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D pelvisTopShape = new FrameCapsule3D(pelvisFrame, 0.2 * modelScale, 0.1 * modelScale);
         pelvisTopShape.getPosition().set(-0.005, 0.0, -0.076);
         pelvisTopShape.getPosition().scale(modelScale);
         pelvisTopShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisTopShape));

         FrameCapsule3D pelvisFrontShape = new FrameCapsule3D(pelvisFrame, 0.19 * modelScale, 0.1 * modelScale);
         pelvisFrontShape.getPosition().set(0.025, 0.0, -0.154);
         pelvisFrontShape.getPosition().scale(modelScale);
         pelvisFrontShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisFrontShape));

         FrameCapsule3D pelvisBackShape = new FrameCapsule3D(pelvisFrame, 0.19 * modelScale, 0.1 * modelScale);
         pelvisBackShape.getPosition().set(-0.015, 0.0, -0.154);
         pelvisBackShape.getPosition().scale(modelScale);
         pelvisBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisBackShape));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         { // Hip
            JointBasics hipYaw = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), multiBodySystem);
            MovingReferenceFrame hipYawFrame = hipYaw.getFrameAfterJoint();
            FrameSphere3D hipShape = new FrameSphere3D(hipYawFrame, 0.13 * modelScale);
            hipShape.getPosition().set(0.0, 0.0, -0.03);
            hipShape.getPosition().scale(modelScale);
            collidables.add(new Collidable(hipYaw.getPredecessor(), collisionMask, collisionGroup, hipShape));
         }

         { // Thigh
            RigidBodyBasics thigh = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem)
                                                       .getPredecessor();
            MovingReferenceFrame thighFrame = thigh.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D thighUpperShape = new FrameCapsule3D(thighFrame, 0.2 * modelScale, 0.1 * modelScale);
            thighUpperShape.getPosition().set(0.0195, robotSide.negateIfRightSide(0.086), -0.093);
            thighUpperShape.getPosition().scale(modelScale);
            thighUpperShape.getAxis().set(new Vector3D(-0.15, robotSide.negateIfRightSide(-0.05), 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighUpperShape));

            FrameCapsule3D thighFrontShape = new FrameCapsule3D(thighFrame, 0.15 * modelScale, 0.095 * modelScale);
            thighFrontShape.getPosition().set(0.0424, robotSide.negateIfRightSide(0.081), -0.258);
            thighFrontShape.getPosition().scale(modelScale);
            thighFrontShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape));

            FrameCapsule3D thighLowerShape = new FrameCapsule3D(thighFrame, 0.25 * modelScale, 0.09 * modelScale);
            thighLowerShape.getPosition().set(0.017, robotSide.negateIfRightSide(0.091), -0.288);
            thighLowerShape.getPosition().scale(modelScale);
            thighLowerShape.getAxis().set(new Vector3D(0.1, robotSide.negateIfRightSide(0.05), 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighLowerShape));
         }

         { // Shin
            RigidBodyBasics shin = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem).getSuccessor();
            MovingReferenceFrame shinFrame = shin.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D shinShape = new FrameCapsule3D(shinFrame, 0.3 * modelScale, 0.08 * modelScale);
            shinShape.getPosition().set(0.008, 0.0, -0.189);
            shinShape.getPosition().scale(modelScale);
            shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
         }

         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();

            // Using a STP box so the sole is slightly rounded allowing for continuous and smooth contact with the ground.
            Vector3D footSize = new Vector3D(0.275, 0.16, 0.095);
            if (createForSimulation)
            {
               FrameSTPBox3D footShape = new FrameSTPBox3D(ankleRollFrame, footSize);
               footShape.getSize().scale(modelScale);
               footShape.setMargins(1.0e-5, 4.0e-4);
               footShape.getPosition().set(0.044, 0.0, -0.042);
               footShape.getPosition().scale(modelScale);
               collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footShape));
            }
            else
            {
               FrameBox3D footShape = new FrameBox3D(ankleRollFrame, footSize);
               footShape.getSize().scale(modelScale);
               footShape.getPosition().set(0.044, 0.0, -0.042);
               footShape.getPosition().scale(modelScale);
               collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footShape));
            }
         }
      }

      return collidables;
   }
}
