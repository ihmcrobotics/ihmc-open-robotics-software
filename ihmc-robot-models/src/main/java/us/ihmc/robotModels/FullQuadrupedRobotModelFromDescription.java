package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class FullQuadrupedRobotModelFromDescription extends FullRobotModelFromDescription implements FullQuadrupedRobotModel
{
   private final BiMap<QuadrupedJointName, OneDoFJointBasics> jointNameOneDoFJointBiMap = HashBiMap.create();
   @Deprecated
   private final Map<QuadrupedJointName, JointLimit> jointLimits = new EnumMap<>(QuadrupedJointName.class);
   private final Map<OneDoFJointBasics, JointLimitData> jointLimitData = new HashMap<>();
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames = new QuadrantDependentList<>();

   private QuadrantDependentList<EnumMap<LegJointName, OneDoFJointBasics>> legJointMaps;
   private QuadrantDependentList<ArrayList<OneDoFJointBasics>> legJointLists;

   private QuadrantDependentList<RigidBodyBasics> feet;
   private boolean initialized = false;

   private final RobotQuadrant[] robotQuadrants;

   public FullQuadrupedRobotModelFromDescription(RobotDescription description, QuadrupedJointNameMap sdfJointNameMap, String[] sensorLinksToTrack,
                                                 Map<QuadrupedJointName, JointLimit> jointLimits)
   {
      this(RobotQuadrant.values, description, sdfJointNameMap, sensorLinksToTrack);

      this.jointLimits.putAll(jointLimits);
   }

   public FullQuadrupedRobotModelFromDescription(RobotDescription description, QuadrupedJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      this(RobotQuadrant.values, description, sdfJointNameMap, sensorLinksToTrack);
   }

   public FullQuadrupedRobotModelFromDescription(RobotQuadrant[] robotQuadrants, RobotDescription description, QuadrupedJointNameMap sdfJointNameMap,
                                                 String[] sensorLinksToTrack)
   {
      super(description, sdfJointNameMap, sensorLinksToTrack);

      this.robotQuadrants = robotQuadrants;

      for (OneDoFJointBasics oneDoFJoint : getOneDoFJoints())
      {
         QuadrupedJointName quadrupedJointName = sdfJointNameMap.getJointNameForSDFName(oneDoFJoint.getName());
         jointNameOneDoFJointBiMap.put(quadrupedJointName, oneDoFJoint);

         // Assign default joint limits
         JointLimit jointLimit = new JointLimit(oneDoFJoint);
         jointLimits.put(quadrupedJointName, jointLimit);

         jointLimitData.put(oneDoFJoint, new JointLimitData(oneDoFJoint));
      }

      for (RobotQuadrant robotQuadrant : robotQuadrants)
      {
         RigidBodyTransform soleToParentTransform = sdfJointNameMap.getSoleToParentFrameTransform(robotQuadrant);
         MovingReferenceFrame soleFrame = MovingReferenceFrame
               .constructFrameFixedInParent(robotQuadrant.toString() + "SoleFrame", getEndEffectorFrame(robotQuadrant, LimbName.LEG), soleToParentTransform);
         soleFrames.put(robotQuadrant, soleFrame);
      }
   }

   private boolean hasQuadrant(RobotQuadrant quadrant)
   {
      for (RobotQuadrant robotQuadrant : robotQuadrants)
      {
         if (robotQuadrant == quadrant)
            return true;
      }

      return false;
   }

   @Override
   protected void mapRigidBody(JointDescription joint, OneDoFJointBasics inverseDynamicsJoint, RigidBodyBasics rigidBody)
   {
      initializeLists();

      super.mapRigidBody(joint, inverseDynamicsJoint, rigidBody);

      QuadrupedJointNameMap jointMap = (QuadrupedJointNameMap) sdfJointNameMap;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotQuadrant);

         if (jointBeforeFootName != null && jointBeforeFootName.equals(joint.getName()))
         {
            feet.set(robotQuadrant, rigidBody);
         }
      }

      JointRole jointRole = sdfJointNameMap.getJointRole(joint.getName());
      if (jointRole != null)
      {
         switch (jointRole)
         {
         case LEG:
            ImmutablePair<RobotQuadrant, LegJointName> legJointName = jointMap.getLegJointName(joint.getName());
            legJointMaps.get(legJointName.getLeft()).put(legJointName.getRight(), inverseDynamicsJoint);
            legJointLists.get(legJointName.getLeft()).add(inverseDynamicsJoint);
            break;
         }
      }
   }

   private void initializeLists()
   {
      if (!initialized)
      {
         legJointMaps = QuadrantDependentList.createListOfEnumMaps(LegJointName.class);
         legJointLists = QuadrantDependentList.createListOfArrayLists();

         feet = new QuadrantDependentList<>();
         initialized = true;
      }
   }

   @Override
   public OneDoFJointBasics getLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      if (hasQuadrant(robotQuadrant))
         return legJointMaps.get(robotQuadrant).get(legJointName);
      else
         return null;
   }

   /* (non-Javadoc)
       * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getFoot(us.ihmc.robotics.robotSide.RobotQuadrant)
       */
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
   public RigidBodyBasics getBody()
   {
      return getRootBody();
   }

   @Override
   public MovingReferenceFrame getEndEffectorFrame(RobotQuadrant robotQuadrant, LimbName limbName)
   {
      if (hasQuadrant(robotQuadrant))
         return getEndEffector(robotQuadrant, limbName).getParentJoint().getFrameAfterJoint();
      else
         return null;
   }

   @Override
   public SegmentDependentList<RobotQuadrant, MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getNameForOneDoFJoint(us.ihmc.robotics.screwTheory.OneDoFJoint)
    */
   @Override
   public QuadrupedJointName getNameForOneDoFJoint(OneDoFJointBasics oneDoFJoint)
   {
      return jointNameOneDoFJointBiMap.inverse().get(oneDoFJoint);
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getJointLimit(us.ihmc.modelFileLoaders.SdfLoader.partNames.QuadrupedJointName)
    */
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
   public List<OneDoFJointBasics> getLegJointsList(RobotQuadrant robotQuadrant)
   {
      return legJointLists.get(robotQuadrant);
   }
}
