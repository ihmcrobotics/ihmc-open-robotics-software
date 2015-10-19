package us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class JointAnglesWriter
{

   private String name;
   private SDFRobot robot;
   protected final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
   protected ImmutablePair<FloatingJoint, SixDoFJoint> rootJointPair;

   public JointAnglesWriter(SDFRobot robot, FullRobotModel fullRobotModel)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;

      setFullRobotModel(fullRobotModel);
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      revoluteJoints.clear();
      OneDoFJoint[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();

      for (OneDoFJoint revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
      }

      rootJointPair = new ImmutablePair<FloatingJoint, SixDoFJoint>(robot.getRootJoint(), fullRobotModel.getRootJoint());
   }

   public String getName()
   {
      return name;
   }
   
   public void write()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
      }
   }

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
         pinJoint.setQd(revoluteJoint.getQd());
         pinJoint.setQdd(revoluteJoint.getQdd());
      }
      
      FloatingJoint floatingJoint = rootJointPair.getLeft();
      SixDoFJoint sixDoFJoint = rootJointPair.getRight();
      
      RigidBodyTransform transform = sixDoFJoint.getJointTransform3D();
      floatingJoint.setRotationAndTranslation(transform);
   }
   
}