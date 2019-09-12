package us.ihmc.avatar.jointAnglesWriter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

/**
 * This class needs to extend OutputWriter or use some other common class. @dcalvert @sbertrand
 */
public class JointAnglesWriter
{
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>> oneDoFJointPairList = new ArrayList<>();
   private final ImmutablePair<FloatingJoint, FloatingJointBasics> rootJointPair;

   private boolean writeJointVelocities = true;
   private boolean writeJointAccelerations = true;

   public JointAnglesWriter(Robot robot, FullRobotModel fullRobotModel)
   {
      this(robot, fullRobotModel.getRootJoint(), fullRobotModel.getOneDoFJoints());
   }

   public JointAnglesWriter(Robot robot, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints)
   {
      oneDoFJointPairList.clear();
      Map<String, OneDegreeOfFreedomJoint> scsJointMap = new HashMap<>();
      ArrayList<OneDegreeOfFreedomJoint> scsOnDoFJointList = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(scsOnDoFJointList);
      scsOnDoFJointList.forEach(joint -> scsJointMap.put(joint.getName(), joint));

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         String name = joint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = scsJointMap.get(name);
         if (oneDoFJoint == null)
            throw new RuntimeException("Could not find the SCS joint with the name: " + name);

         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>(oneDoFJoint,
                                                                                                                                             joint);
         this.oneDoFJointPairList.add(jointPair);
      }

      FloatingJoint scsRootJoint;
      if (robot.getRootJoints().get(0) instanceof FloatingJoint)
         scsRootJoint = (FloatingJoint) robot.getRootJoints().get(0);
      else
         scsRootJoint = null;

      if (scsRootJoint == null && rootJoint != null)
         throw new RuntimeException("A " + FloatingJointBasics.class.getSimpleName() + " was provided but there is no " + FloatingJoint.class.getSimpleName());
      if (rootJoint == null && scsRootJoint != null)
         throw new RuntimeException("A " + FloatingJoint.class.getSimpleName() + " was provided but there is no " + FloatingJointBasics.class.getSimpleName());

      if (rootJoint != null)
         rootJointPair = new ImmutablePair<>(scsRootJoint, rootJoint);
      else
         rootJointPair = null;
   }

   public void setWriteJointVelocities(boolean writeJointVelocities)
   {
      this.writeJointVelocities = writeJointVelocities;
   }

   public void setWriteJointAccelerations(boolean writeJointAccelerations)
   {
      this.writeJointAccelerations = writeJointAccelerations;
   }

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair : oneDoFJointPairList)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
         if (writeJointVelocities)
            pinJoint.setQd(revoluteJoint.getQd());
         if (writeJointAccelerations)
            pinJoint.setQdd(revoluteJoint.getQdd());
      }

      if (rootJointPair != null)
      {
         FloatingJoint floatingJoint = rootJointPair.getLeft();
         FloatingJointBasics sixDoFJoint = rootJointPair.getRight();

         RigidBodyTransform transform = new RigidBodyTransform();
         sixDoFJoint.getJointConfiguration(transform);
         floatingJoint.setRotationAndTranslation(transform);

         if (writeJointVelocities)
         {
            FixedFrameTwistBasics jointTwist = sixDoFJoint.getJointTwist();
            floatingJoint.setAngularVelocityInBody(jointTwist.getAngularPart());
            linearVelocity.setMatchingFrame(jointTwist.getLinearPart());
            floatingJoint.setVelocity(linearVelocity);
         }

         if (writeJointAccelerations)
         {
            FixedFrameTwistBasics jointTwist = sixDoFJoint.getJointTwist();
            FixedFrameSpatialAccelerationBasics jointAcceleration = sixDoFJoint.getJointAcceleration();
            floatingJoint.setAngularAccelerationInBody(jointAcceleration.getAngularPart());
            jointAcceleration.getLinearAccelerationAtBodyOrigin(jointTwist, linearAcceleration);
            linearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
            floatingJoint.setAcceleration(linearAcceleration);
         }
      }
   }
}