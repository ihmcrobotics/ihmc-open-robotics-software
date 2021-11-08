package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FullHumanoidRobotModelFromDescription extends FullRobotModelFromDescription implements FullHumanoidRobotModel
{
   private SideDependentList<EnumMap<ArmJointName, OneDoFJointBasics>> armJointLists;
   private SideDependentList<EnumMap<LegJointName, OneDoFJointBasics>> legJointLists;

   private RigidBodyBasics[] endEffectors = new RigidBodyBasics[4];
   private SideDependentList<RigidBodyBasics> feet;
   private SideDependentList<RigidBodyBasics> hands;

   private RigidBodyBasics chest;

   private boolean initialized = false;

   private final SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> handControlFrames = new SideDependentList<>();
   private final OneDoFJointBasics[] oneDoFJointsExcludingHands;

   private HumanoidJointNameMap humanoidJointNameMap;

   // copy constructor
   public FullHumanoidRobotModelFromDescription(FullHumanoidRobotModelFromDescription modelToCopy)
   {
      this(modelToCopy.description, modelToCopy.humanoidJointNameMap, modelToCopy.sensorLinksToTrack);
   }

   public FullHumanoidRobotModelFromDescription(RobotDescription description, HumanoidJointNameMap jointNameMap)
   {
      this(description, jointNameMap, new String[0]);
   }

   public FullHumanoidRobotModelFromDescription(RobotDescription description, HumanoidJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      super(description, sdfJointNameMap, sensorLinksToTrack);

      humanoidJointNameMap = sdfJointNameMap;

      FloatingJointDescription rootJointDescription = (FloatingJointDescription) description.getRootJoints().get(0);
      if (!rootJointDescription.getName().equals(sdfJointNameMap.getPelvisName()))
      {
         throw new RuntimeException("Pelvis joint is assumed to be the root joint. rootJointDescription.getName() = " + rootJointDescription.getName() + ", sdfJointNameMap.getPelvisName() = " + sdfJointNameMap.getPelvisName());
      }


      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleTransform = sdfJointNameMap.getSoleToParentFrameTransform(robotSide);
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         MovingReferenceFrame soleFrame = MovingReferenceFrame.constructFrameFixedInParent(sidePrefix + "Sole", getEndEffectorFrame(robotSide, LimbName.LEG), soleToAnkleTransform);
         soleFrames.put(robotSide, soleFrame);

         RigidBodyTransform handControlFrameToWristTransform = sdfJointNameMap.getHandControlFrameToWristTransform(robotSide);

         if (handControlFrameToWristTransform != null)
         {
            MovingReferenceFrame attachmentPlateFrame = MovingReferenceFrame.constructFrameFixedInParent(sidePrefix + "HandControlFrame", getEndEffectorFrame(robotSide, LimbName.ARM),
                  handControlFrameToWristTransform);
            handControlFrames.put(robotSide, attachmentPlateFrame);
         }
         else
         {
            handControlFrames.put(robotSide, null);
         }
      }

      int index = 0;
      for (RobotSide robotSide : RobotSide.values)
      {
         endEffectors[index] = feet.get(robotSide);
         endEffectors[index + 2] = hands.get(robotSide);
         index++;
      }

      oneDoFJointsExcludingHands = getAllJointsExcludingHands();
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics[] getControllableOneDoFJoints()
   {
      return oneDoFJointsExcludingHands;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics getLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointLists.get(robotSide).get(legJointName);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getChest()
   {
      return chest;
   }


   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics getArmJoint(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointLists.get(robotSide).get(armJointName);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getFoot(RobotSide robotSide)
   {
      return feet.get(robotSide);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getHand(RobotSide robotSide)
   {
      return hands.get(robotSide);
   }

   /** {@inheritDoc} */
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
   public SideDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
   {
      return handControlFrames.get(robotSide);
   }

   @Override
   protected void mapRigidBody(JointDescription joint, OneDoFJointBasics inverseDynamicsJoint, RigidBodyBasics rigidBody)
   {
      initializeLists();

      LinkDescription childLink = joint.getLink();
      HumanoidJointNameMap humanoidJointNameMap = (HumanoidJointNameMap) sdfJointNameMap;

      if (rigidBody.getName().equals(humanoidJointNameMap.getChestName()))
      {
         chest = rigidBody;
      }

      if (rigidBody.getName().equals(humanoidJointNameMap.getHeadName()))
      {
         head = rigidBody;
      }
      ImmutablePair<RobotSide, LimbName> limbSideAndName = humanoidJointNameMap.getLimbName(childLink.getName());
      if (limbSideAndName != null)
      {
         RobotSide limbSide = limbSideAndName.getLeft();
         LimbName limbName = limbSideAndName.getRight();
         switch (limbName)
         {
         case ARM:
            hands.put(limbSide, rigidBody);
            break;
         case LEG:
            feet.put(limbSide, rigidBody);
            break;
         }
      }

      JointRole jointRole = sdfJointNameMap.getJointRole(joint.getName());
      if (jointRole != null)
      {
         switch (jointRole)
         {
         //TODO: Should armJointLists use legJoingName.first or armJointName.first?? looks backwards
         case ARM:
            ImmutablePair<RobotSide, ArmJointName> armJointName = humanoidJointNameMap.getArmJointName(joint.getName());
            armJointLists.get(armJointName.getLeft()).put(armJointName.getRight(), inverseDynamicsJoint);
            break;
         case LEG:
            ImmutablePair<RobotSide, LegJointName> legJointName = humanoidJointNameMap.getLegJointName(joint.getName());
            legJointLists.get(legJointName.getLeft()).put(legJointName.getRight(), inverseDynamicsJoint);
            break;
         case NECK:
            NeckJointName neckJointName = humanoidJointNameMap.getNeckJointName(joint.getName());
            neckJoints.put(neckJointName, inverseDynamicsJoint);
            break;
         case SPINE:
            SpineJointName spineJointName = humanoidJointNameMap.getSpineJointName(joint.getName());
            spineJoints.put(spineJointName, inverseDynamicsJoint);
            break;
         }
      }
   }

   private void initializeLists()
   {
      if (!initialized)
      {
         armJointLists = SideDependentList.createListOfEnumMaps(ArmJointName.class);
         legJointLists = SideDependentList.createListOfEnumMaps(LegJointName.class);

         feet = new SideDependentList<RigidBodyBasics>();
         hands = new SideDependentList<RigidBodyBasics>();
         initialized = true;
      }
   }

   private OneDoFJointBasics[] getAllJointsExcludingHands()
   {
      List<OneDoFJointBasics> joints = new ArrayList<>();
      getOneDoFJoints(joints);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = getHand(robotSide);
         if (hand != null)
         {
            OneDoFJointBasics[] fingerJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(hand), OneDoFJointBasics.class);
            for (OneDoFJointBasics fingerJoint : fingerJoints)
            {
               joints.remove(fingerJoint);
            }
         }
      }
      return joints.toArray(new OneDoFJointBasics[0]);
   }
}
