package us.ihmc.avatar.jointAnglesWriter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

/**
 * This class needs to extend OutputWriter or use some other common class. @dcalvert @sbertrand
 */
public class JointAnglesWriter
{
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> oneDoFJointPairList = new ArrayList<>();
   private final ImmutablePair<FloatingJoint, FloatingInverseDynamicsJoint> rootJointPair;

   public JointAnglesWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel)
   {
      this(robot.getRootJoint(), robot.getOneDegreeOfFreedomJoints(), fullRobotModel.getRootJoint(), fullRobotModel.getOneDoFJoints());
   }

   public JointAnglesWriter(FloatingJoint scsRootJoint, OneDegreeOfFreedomJoint[] scsJoints, FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] oneDoFJoints)
   {
      oneDoFJointPairList.clear();
      Map<String, OneDegreeOfFreedomJoint> scsJointMap = new HashMap<>();
      Arrays.stream(scsJoints).forEach(joint -> scsJointMap.put(joint.getName(), joint));

      for (OneDoFJoint joint : oneDoFJoints)
      {
         String name = joint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = scsJointMap.get(name);
         if (oneDoFJoint == null)
            throw new RuntimeException("Could not find the SCS joint with the name: " + name);

         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, joint);
         this.oneDoFJointPairList.add(jointPair);
      }

      if (scsRootJoint == null && rootJoint != null)
         throw new RuntimeException("A " + FloatingInverseDynamicsJoint.class.getSimpleName() + " was provided but there is no "
               + FloatingJoint.class.getSimpleName());
      if (rootJoint == null && scsRootJoint != null)
         throw new RuntimeException("A " + FloatingJoint.class.getSimpleName() + " was provided but there is no "
               + FloatingInverseDynamicsJoint.class.getSimpleName());

      if (rootJoint != null)
         rootJointPair = new ImmutablePair<>(scsRootJoint, rootJoint);
      else
         rootJointPair = null;
   }

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : oneDoFJointPairList)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
         pinJoint.setQd(revoluteJoint.getQd());
         pinJoint.setQdd(revoluteJoint.getQdd());
      }

      if (rootJointPair != null)
      {
         FloatingJoint floatingJoint = rootJointPair.getLeft();
         FloatingInverseDynamicsJoint sixDoFJoint = rootJointPair.getRight();

         RigidBodyTransform transform = sixDoFJoint.getJointTransform3D();
         floatingJoint.setRotationAndTranslation(transform);
      }
   }
}