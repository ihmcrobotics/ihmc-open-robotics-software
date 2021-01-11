package us.ihmc.simulationToolkit.outputWriters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class PerfectSimulatedOutputWriter implements OutputWriter
{
   private final String name;
   protected final FloatingRootJointRobot robot;
   protected ImmutablePair<FloatingJoint, FloatingJointBasics> rootJointPair;
   protected final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJointBasics>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>>();
   private final JointDesiredOutputListReadOnly jointDesiredOutputList;

   public PerfectSimulatedOutputWriter(FloatingRootJointRobot robot)
   {
      this(robot, null);
   }

   public PerfectSimulatedOutputWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel)
   {
      this(robot, fullRobotModel, null);
   }

   public PerfectSimulatedOutputWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.jointDesiredOutputList = jointDesiredOutputList;

      if (fullRobotModel != null)
         setFullRobotModel(fullRobotModel);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      revoluteJoints.clear();
      OneDoFJointBasics[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();

      for (OneDoFJointBasics revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJointBasics> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
      }

      rootJointPair = new ImmutablePair<FloatingJoint, FloatingJointBasics>(robot.getRootJoint(), fullRobotModel.getRootJoint());
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   @Override
   public void write()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         double tau;
         if (jointDesiredOutputList != null)
            tau = jointDesiredOutputList.getJointDesiredOutput(revoluteJoint).getDesiredTorque();
         else
            tau = revoluteJoint.getTau();

         pinJoint.setTau(tau);
      }
   }

   public void updateRobotConfigurationBasedOnJointDesiredOutputPositions()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();
         
         double q = jointDesiredOutputList.getJointDesiredOutput(revoluteJoint).getDesiredPosition();
         pinJoint.setQ(q);

         double qd = jointDesiredOutputList.getJointDesiredOutput(revoluteJoint).getDesiredVelocity();
         pinJoint.setQd(qd);

         double qdd = jointDesiredOutputList.getJointDesiredOutput(revoluteJoint).getDesiredAcceleration();
         pinJoint.setQdd(qdd);
      }
   }

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
      }

      FloatingJoint floatingJoint = rootJointPair.getLeft();
      FloatingJointBasics sixDoFJoint = rootJointPair.getRight();

      RigidBodyTransform transform = new RigidBodyTransform();
      sixDoFJoint.getJointConfiguration(transform);
      floatingJoint.setRotationAndTranslation(transform);
   }
}
