package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FullQuadrupedRobotModelFromDescription extends FullRobotModelFromDescription implements FullQuadrupedRobotModel
{
   private final BiMap<QuadrupedJointName, OneDoFJoint> jointNameOneDoFJointBiMap = HashBiMap.create();
   private final Map<QuadrupedJointName, JointLimit> jointLimits = new EnumMap<>(QuadrupedJointName.class);
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames = new QuadrantDependentList<>();

   private QuadrantDependentList<EnumMap<LegJointName, OneDoFJoint>> legJointMaps;
   private QuadrantDependentList<ArrayList<OneDoFJoint>> legJointLists;

   private QuadrantDependentList<RigidBody> feet;

   private boolean initialized = false;

   public FullQuadrupedRobotModelFromDescription(RobotDescription description, QuadrupedJointNameMap sdfJointNameMap, String[] sensorLinksToTrack,
         Map<QuadrupedJointName, JointLimit> jointLimits)
   {
      this(description, sdfJointNameMap, sensorLinksToTrack);

      this.jointLimits.putAll(jointLimits);
   }

   public FullQuadrupedRobotModelFromDescription(RobotDescription description, QuadrupedJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      super(description, sdfJointNameMap, sensorLinksToTrack);

      for (OneDoFJoint oneDoFJoint : getOneDoFJoints())
      {
         QuadrupedJointName quadrupedJointName = sdfJointNameMap.getJointNameForSDFName(oneDoFJoint.getName());
         jointNameOneDoFJointBiMap.put(quadrupedJointName, oneDoFJoint);

         // Assign default joint limits
         JointLimit jointLimit = new JointLimit(oneDoFJoint);
         jointLimits.put(quadrupedJointName, jointLimit);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform soleToParentTransform = sdfJointNameMap.getSoleToParentFrameTransform(robotQuadrant);
         MovingReferenceFrame soleFrame = MovingReferenceFrame.constructFrameFixedInParent(robotQuadrant.toString() + "SoleFrame", getEndEffectorFrame(robotQuadrant, LimbName.LEG), soleToParentTransform);
         soleFrames.put(robotQuadrant, soleFrame);
      }
   }

   @Override
   protected void mapRigidBody(JointDescription joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      initializeLists();

      super.mapRigidBody(joint, inverseDynamicsJoint, rigidBody);

      QuadrupedJointNameMap jointMap = (QuadrupedJointNameMap) sdfJointNameMap;
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotQuadrant);

         if(jointBeforeFootName.equals(joint.getName()))
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
   public MovingReferenceFrame getFrameAfterLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return getLegJoint(robotQuadrant, legJointName).getFrameAfterJoint();
   }

   @Override
   public OneDoFJoint getLegJoint(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      return legJointMaps.get(robotQuadrant).get(legJointName);
   }

   /* (non-Javadoc)
       * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getFoot(us.ihmc.robotics.robotSide.RobotQuadrant)
       */
   @Override
   public RigidBody getFoot(RobotQuadrant robotQuadrant)
   {
      return feet.get(robotQuadrant);
   }

   @Override
   public RigidBody getEndEffector(RobotQuadrant robotQuadrant, LimbName limbName)
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
   public RigidBody getBodyLink()
   {
      return getRootLink();
   }

   @Override
   public MovingReferenceFrame getEndEffectorFrame(RobotQuadrant robotQuadrant, LimbName limbName)
   {
      return getEndEffector(robotQuadrant, limbName).getParentJoint().getFrameAfterJoint();
   }

   @Override
   public MovingReferenceFrame getSoleFrame(RobotQuadrant robotQuadrant)
   {
      return soleFrames.get(robotQuadrant);
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
   public QuadrupedJointName getNameForOneDoFJoint(OneDoFJoint oneDoFJoint)
   {
      return jointNameOneDoFJointBiMap.inverse().get(oneDoFJoint);
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getJointLimit(us.ihmc.modelFileLoaders.SdfLoader.partNames.QuadrupedJointName)
    */
   @Override
   public JointLimit getJointLimit(QuadrupedJointName jointName)
   {
      return jointLimits.get(jointName);
   }

   @Override
   public List<OneDoFJoint> getLegJointsList(RobotQuadrant robotQuadrant)
   {
      return legJointLists.get(robotQuadrant);
   }
}
