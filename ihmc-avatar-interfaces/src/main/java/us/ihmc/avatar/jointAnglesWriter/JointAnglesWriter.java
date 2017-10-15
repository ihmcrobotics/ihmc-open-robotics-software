package us.ihmc.avatar.jointAnglesWriter;

import java.util.ArrayList;
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
import us.ihmc.simulationconstructionset.Robot;

/**
 * This class needs to extend OutputWriter or use some other common class. @dcalvert @sbertrand
 */
public class JointAnglesWriter
{
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> oneDoFJointPairList = new ArrayList<>();
   private final ImmutablePair<FloatingJoint, FloatingInverseDynamicsJoint> rootJointPair;

   public JointAnglesWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel)
   {
      this(robot, fullRobotModel.getRootJoint(), fullRobotModel.getOneDoFJoints());
   }

   public JointAnglesWriter(Robot robot, FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] oneDoFJoints)
   {
      oneDoFJointPairList.clear();
      Map<String, OneDegreeOfFreedomJoint> scsJointMap = new HashMap<>();
      ArrayList<OneDegreeOfFreedomJoint> scsOnDoFJointList = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(scsOnDoFJointList);
      scsOnDoFJointList.forEach(joint -> scsJointMap.put(joint.getName(), joint));

      for (OneDoFJoint joint : oneDoFJoints)
      {
         String name = joint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = scsJointMap.get(name);
         if (oneDoFJoint == null)
            throw new RuntimeException("Could not find the SCS joint with the name: " + name);

         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, joint);
         this.oneDoFJointPairList.add(jointPair);
      }

      FloatingJoint scsRootJoint;
      if (robot.getRootJoints().get(0) instanceof FloatingJoint)
         scsRootJoint = (FloatingJoint) robot.getRootJoints().get(0);
      else
         scsRootJoint = null;

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