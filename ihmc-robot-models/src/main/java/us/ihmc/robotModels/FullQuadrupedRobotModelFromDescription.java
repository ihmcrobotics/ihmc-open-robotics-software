package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FullQuadrupedRobotModelFromDescription extends FullRobotModelFromDescription implements FullQuadrupedRobotModel
{
   private final BiMap<QuadrupedJointName, OneDoFJoint> jointNameOneDoFJointBiMap = HashBiMap.create();
   private final QuadrantDependentList<List<OneDoFJoint>> legOneDoFJoints = new QuadrantDependentList<>();
   private final Map<QuadrupedJointName, JointLimit> jointLimits = new EnumMap<>(QuadrupedJointName.class);

   private QuadrantDependentList<RigidBody> feet;

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

         // Map leg names to quadrants
         if (quadrupedJointName.getRole() == JointRole.LEG)
         {
            RobotQuadrant quadrant = quadrupedJointName.getQuadrant();
            if (legOneDoFJoints.get(quadrant) == null)
            {
               legOneDoFJoints.set(quadrant, new ArrayList<OneDoFJoint>());
            }

            legOneDoFJoints.get(quadrant).add(oneDoFJoint);
         }

         // Assign default joint limits
         JointLimit jointLimit = new JointLimit(oneDoFJoint);
         jointLimits.put(quadrupedJointName, jointLimit);
      }
   }

   @Override
   protected void mapRigidBody(JointDescription joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      if(feet == null)
      {
         feet = new QuadrantDependentList<RigidBody>();
      }

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
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getFoot(us.ihmc.robotics.robotSide.RobotQuadrant)
    */
   @Override
   public RigidBody getFoot(RobotQuadrant robotQuadrant)
   {
      return feet.get(robotQuadrant);
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getLegOneDoFJoints(us.ihmc.robotics.robotSide.RobotQuadrant)
    */
   @Override
   public List<OneDoFJoint> getLegOneDoFJoints(RobotQuadrant quadrant)
   {
      return legOneDoFJoints.get(quadrant);
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getOneDoFJointBeforeFoot(us.ihmc.robotics.robotSide.RobotQuadrant)
    */
   @Override
   public OneDoFJoint getOneDoFJointBeforeFoot(RobotQuadrant quadrant)
   {
      return (OneDoFJoint) getFoot(quadrant).getParentJoint();
   }

   /* (non-Javadoc)
    * @see us.ihmc.modelFileLoaders.SdfLoader.FullQuadrupedRobotModel#getOneDoFJointByName(us.ihmc.modelFileLoaders.SdfLoader.partNames.QuadrupedJointName)
    */
   @Override
   public OneDoFJoint getOneDoFJointByName(QuadrupedJointName name)
   {
      return jointNameOneDoFJointBiMap.get(name);
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
}
