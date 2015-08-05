package us.ihmc.SdfLoader;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class SDFPerfectSimulatedOutputWriter 
{
   private final String name;
   protected final SDFRobot robot;
   protected ImmutablePair<FloatingJoint, SixDoFJoint> rootJointPair;
   protected final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
   
   public SDFPerfectSimulatedOutputWriter(SDFRobot robot)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
   }
   
   public SDFPerfectSimulatedOutputWriter(SDFRobot robot, FullRobotModel fullRobotModel)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;

      setFullRobotModel(fullRobotModel);
   }

   public void initialize()
   {
   }
   
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      revoluteJoints.clear();
      OneDoFJoint[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();
      
      for (OneDoFJoint revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);
         
         ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
      }
      
      rootJointPair = new ImmutablePair<FloatingJoint, SixDoFJoint>(robot.getRootJoint(), fullRobotModel.getRootJoint());
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void write()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         pinJoint.setTau(revoluteJoint.getTau());
      }
   }

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
      }
      
      FloatingJoint floatingJoint = rootJointPair.getLeft();
      SixDoFJoint sixDoFJoint = rootJointPair.getRight();
      
      RigidBodyTransform transform = sixDoFJoint.getJointTransform3D();
      floatingJoint.setRotationAndTranslation(transform);
   }
}
