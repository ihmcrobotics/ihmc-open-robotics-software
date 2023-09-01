package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import us.ihmc.euclid.referenceFrame.FrameNameRestrictionLevel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class FullQuadrupedRobotModelWrapper extends FullRobotModelWrapper implements FullQuadrupedRobotModel
{
   private BiMap<QuadrupedJointName, OneDoFJointBasics> jointNameOneDoFJointBiMap;
   @Deprecated
   private Map<QuadrupedJointName, JointLimit> jointLimits;
   private Map<OneDoFJointBasics, JointLimitData> jointLimitData;
   private QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private QuadrantDependentList<EnumMap<LegJointName, OneDoFJointBasics>> legJointMaps;
   private QuadrantDependentList<ArrayList<OneDoFJointBasics>> legJointLists;

   private QuadrantDependentList<RigidBodyBasics> feet;

   public FullQuadrupedRobotModelWrapper(RobotDefinition robotDefinition, QuadrupedJointNameMap jointNameMap, Map<QuadrupedJointName, JointLimit> jointLimits)
   {
      this(robotDefinition, jointNameMap);
      this.jointLimits.putAll(jointLimits);
   }

   public FullQuadrupedRobotModelWrapper(RobotDescription robotDescription, QuadrupedJointNameMap jointNameMap, Map<QuadrupedJointName, JointLimit> jointLimits)
   {
      this(robotDescription, jointNameMap);
      this.jointLimits.putAll(jointLimits);
   }

   public FullQuadrupedRobotModelWrapper(RobotDefinition robotDefinition, QuadrupedJointNameMap jointNameMap)
   {
      this(robotDefinition, jointNameMap, true);
   }

   public FullQuadrupedRobotModelWrapper(RobotDefinition robotDefinition, QuadrupedJointNameMap jointNameMap, boolean enforceUniqueReferenceFrames)
   {
      super(robotDefinition.newInstance(ReferenceFrame.getWorldFrame()), enforceUniqueReferenceFrames);
      setupQuadrupedJointNameMap(jointNameMap);
      setupRobotDefinition(robotDefinition);
   }

   public FullQuadrupedRobotModelWrapper(RobotDescription robotDescription, QuadrupedJointNameMap jointNameMap)
   {
      this(robotDescription, jointNameMap, true);
   }

   public FullQuadrupedRobotModelWrapper(RobotDescription robotDescription, QuadrupedJointNameMap jointNameMap, boolean enforceUniqueReferenceFrames)
   {
      super(instantiateRobot(robotDescription, ReferenceFrame.getWorldFrame()), enforceUniqueReferenceFrames);
      setupQuadrupedJointNameMap(jointNameMap);
      setupRobotDescription(robotDescription);
   }

   protected void setupQuadrupedJointNameMap(QuadrupedJointNameMap jointNameMap)
   {
      super.setupJointNameMap(jointNameMap);

      jointNameOneDoFJointBiMap = HashBiMap.create();
      jointLimits = new EnumMap<>(QuadrupedJointName.class);
      jointLimitData = new HashMap<>();

      for (OneDoFJointBasics oneDoFJoint : getOneDoFJoints())
      {
         QuadrupedJointName quadrupedJointName = jointNameMap.getJointNameForSDFName(oneDoFJoint.getName());
         jointNameOneDoFJointBiMap.put(quadrupedJointName, oneDoFJoint);

         // Assign default joint limits
         JointLimit jointLimit = new JointLimit(oneDoFJoint);
         jointLimits.put(quadrupedJointName, jointLimit);

         jointLimitData.put(oneDoFJoint, new JointLimitData(oneDoFJoint));
      }

      feet = new QuadrantDependentList<>();
      soleFrames = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         JointBasics jointBeforeFoot = MultiBodySystemTools.findJoint(getElevator(), jointNameMap.getJointBeforeFootName(robotQuadrant));
         if (jointBeforeFoot != null)
            feet.put(robotQuadrant, jointBeforeFoot.getSuccessor());

         RigidBodyTransform soleToParentTransform = jointNameMap.getSoleToParentFrameTransform(robotQuadrant);
         MovingReferenceFrame soleFrame = MovingReferenceFrame.constructFrameFixedInParent(robotQuadrant.toString() + "SoleFrame",
                                                                                           getEndEffectorFrame(robotQuadrant, LimbName.LEG),
                                                                                           soleToParentTransform);
         soleFrames.put(robotQuadrant, soleFrame);
      }

      legJointMaps = QuadrantDependentList.createListOfEnumMaps(LegJointName.class);
      legJointLists = QuadrantDependentList.createListOfArrayLists();

      for (OneDoFJointBasics oneDoFJoint : getOneDoFJoints())
      {
         String jointName = oneDoFJoint.getName();
         JointRole jointRole = jointNameMap.getJointRole(jointName);
         if (jointRole == null)
            continue;
         switch (jointRole)
         {
            case LEG:
               ImmutablePair<RobotQuadrant, LegJointName> legJointName = jointNameMap.getLegJointName(jointName);
               legJointMaps.get(legJointName.getLeft()).put(legJointName.getRight(), oneDoFJoint);
               legJointLists.get(legJointName.getLeft()).add(oneDoFJoint);
               break;
            default:
               break;
         }
      }
   }

   @Override
   public OneDoFJointBasics getLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return legJointMaps.get(robotQuadrant).get(legJointName);
   }

   @Override
   public RigidBodyBasics getFoot(RobotQuadrant robotQuadrant)
   {
      return feet.get(robotQuadrant);
   }

   @Override
   public RigidBodyBasics getEndEffector(RobotQuadrant robotQuadrant, LimbName limbName)
   {
      switch (limbName)
      {
         case LEG:
            return feet.get(robotQuadrant);
         default:
            throw new RuntimeException("Unknown end effector");
      }
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   @Deprecated
   @Override
   public JointLimit getJointLimit(QuadrupedJointName jointName)
   {
      return jointLimits.get(jointName);
   }

   @Override
   public JointLimitData getJointLimitData(OneDoFJointBasics joint)
   {
      return jointLimitData.get(joint);
   }

   @Override
   public QuadrupedJointName getNameForOneDoFJoint(OneDoFJointBasics oneDoFJoint)
   {
      return jointNameOneDoFJointBiMap.inverse().get(oneDoFJoint);
   }

   @Override
   public List<OneDoFJointBasics> getLegJointsList(RobotQuadrant robotQuadrant)
   {
      return legJointLists.get(robotQuadrant);
   }

   @Override
   public RigidBodyBasics getBody()
   {
      return getRootBody();
   }

}
