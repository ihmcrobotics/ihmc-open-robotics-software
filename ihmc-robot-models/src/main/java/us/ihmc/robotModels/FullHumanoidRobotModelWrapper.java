package us.ihmc.robotModels;

import java.util.Arrays;
import java.util.EnumMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class FullHumanoidRobotModelWrapper extends FullRobotModelWrapper implements FullHumanoidRobotModel
{
   private HumanoidJointNameMap jointNameMap;
   private RigidBodyBasics chest;
   private SideDependentList<RigidBodyBasics> feet;
   private SideDependentList<RigidBodyBasics> hands;
   private SideDependentList<EnumMap<LegJointName, OneDoFJointBasics>> legJointMaps;
   private SideDependentList<EnumMap<ArmJointName, OneDoFJointBasics>> armJointMaps;
   private SideDependentList<MovingReferenceFrame> soleFrames;
   private SideDependentList<MovingReferenceFrame> handControlFrames;
   private OneDoFJointBasics[] controllableOneDoFJoints;

   public FullHumanoidRobotModelWrapper(FullHumanoidRobotModelWrapper other)
   {
      super(other);
      if (other.jointNameMap != null)
         setupHumanoidJointNameMap(other.jointNameMap);
   }

   public FullHumanoidRobotModelWrapper(RobotDefinition robotDefinition, HumanoidJointNameMap jointNameMap)
   {
      this(robotDefinition.newInstance(ReferenceFrame.getWorldFrame()));
      setupHumanoidJointNameMap(jointNameMap);
      setupRobotDefinition(robotDefinition);
   }

   public FullHumanoidRobotModelWrapper(RobotDescription robotDescription, HumanoidJointNameMap jointNameMap)
   {
      this(instantiateRobot(robotDescription, ReferenceFrame.getWorldFrame()));
      setupHumanoidJointNameMap(jointNameMap);
      setupRobotDescription(robotDescription);
   }

   public FullHumanoidRobotModelWrapper(RigidBodyBasics elevator, RobotDefinition robotDefinition, HumanoidJointNameMap jointNameMap)
   {
      this(elevator);
      setupHumanoidJointNameMap(jointNameMap);
      setupRobotDefinition(robotDefinition);
   }

   public FullHumanoidRobotModelWrapper(RigidBodyBasics elevator)
   {
      super(elevator);
   }

   protected void setupHumanoidJointNameMap(HumanoidJointNameMap jointNameMap)
   {
      super.setupJointNameMap(jointNameMap);
      this.jointNameMap = jointNameMap;

      chest = MultiBodySystemTools.findRigidBody(getElevator(), jointNameMap.getChestName());

      feet = new SideDependentList<>();
      hands = new SideDependentList<>();
      soleFrames = new SideDependentList<>();
      handControlFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         feet.put(robotSide, MultiBodySystemTools.findRigidBody(getElevator(), jointNameMap.getFootName(robotSide)));
         hands.put(robotSide, MultiBodySystemTools.findRigidBody(chest, jointNameMap.getHandName(robotSide)));

         RigidBodyTransform soleFrameTransform = jointNameMap.getSoleToParentFrameTransform(robotSide);
         if (soleFrameTransform != null)
         {
            RigidBodyBasics foot = feet.get(robotSide);
            if (foot == null)
               continue;
            soleFrames.put(robotSide,
                           new FixedMovingReferenceFrame(robotSide.getCamelCaseName() + "Sole",
                                                         foot.getParentJoint().getFrameAfterJoint(),
                                                         soleFrameTransform));
         }

         RigidBodyTransform handControlFrameTransform = jointNameMap.getHandControlFrameToWristTransform(robotSide);
         if (handControlFrameTransform != null)
         {
            handControlFrames.put(robotSide,
                                  new FixedMovingReferenceFrame(robotSide.getCamelCaseName() + "HandControlFrame",
                                                                hands.get(robotSide).getParentJoint().getFrameAfterJoint(),
                                                                handControlFrameTransform));
         }
         else if (hands.get(robotSide) != null)
         {
            handControlFrames.put(robotSide, hands.get(robotSide).getParentJoint().getFrameAfterJoint());
         }
      }

      legJointMaps = SideDependentList.createListOfEnumMaps(LegJointName.class);
      armJointMaps = SideDependentList.createListOfEnumMaps(ArmJointName.class);

      for (OneDoFJointBasics oneDoFJoint : getOneDoFJoints())
      {
         String jointName = oneDoFJoint.getName();
         JointRole jointRole = jointNameMap.getJointRole(jointName);
         if (jointRole == null)
            continue;
         switch (jointRole)
         {
            case LEG:
               ImmutablePair<RobotSide, LegJointName> legJointName = jointNameMap.getLegJointName(jointName);
               legJointMaps.get(legJointName.getLeft()).put(legJointName.getRight(), oneDoFJoint);
               break;
            case ARM:
               ImmutablePair<RobotSide, ArmJointName> armJointName = jointNameMap.getArmJointName(jointName);
               armJointMaps.get(armJointName.getLeft()).put(armJointName.getRight(), oneDoFJoint);
               break;
            default:
               break;
         }
      }

      if (getHand(RobotSide.LEFT) == null && getHand(RobotSide.RIGHT) == null)
      {
         controllableOneDoFJoints = getOneDoFJoints();
      }
      else
      {
         controllableOneDoFJoints = Arrays.stream(getOneDoFJoints()).filter(joint ->
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               RigidBodyBasics hand = getHand(robotSide);
               RigidBodyBasics predecessor = joint.getPredecessor();

               if (hand == null)
                  continue;

               if (predecessor == hand || MultiBodySystemTools.isAncestor(predecessor, hand))
                  return false;
            }
            return true;
         }).toArray(OneDoFJointBasics[]::new);
      }
   }

   @Override
   public OneDoFJointBasics[] getControllableOneDoFJoints()
   {
      return controllableOneDoFJoints;
   }

   @Override
   public RigidBodyBasics getChest()
   {
      return chest;
   }

   @Override
   public RigidBodyBasics getFoot(RobotSide robotSide)
   {
      return feet.get(robotSide);
   }

   @Override
   public RigidBodyBasics getHand(RobotSide robotSide)
   {
      return hands.get(robotSide);
   }

   @Override
   public RigidBodyBasics getEndEffector(RobotSide robotSide, LimbName limbName)
   {
      switch (limbName)
      {
         case ARM:
            return hands.get(robotSide);
         case LEG:
            return feet.get(robotSide);
         default:
            throw new RuntimeException("Unknown end effector");
      }
   }

   @Override
   public OneDoFJointBasics getLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointMaps.get(robotSide).get(legJointName);
   }

   @Override
   public OneDoFJointBasics getArmJoint(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointMaps.get(robotSide).get(armJointName);
   }

   @Override
   public SideDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   @Override
   public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
   {
      return handControlFrames.get(robotSide);
   }
}
