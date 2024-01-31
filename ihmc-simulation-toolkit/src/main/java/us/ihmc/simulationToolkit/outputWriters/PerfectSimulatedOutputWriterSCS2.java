package us.ihmc.simulationToolkit.outputWriters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PerfectSimulatedOutputWriterSCS2 implements JointDesiredOutputWriter
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   
   private final Robot robot;
   private JointDesiredOutputListReadOnly jointDesiredOutputList;
   private final List<JointOutputWriter> jointOutputWriters = new ArrayList<>();

   public PerfectSimulatedOutputWriterSCS2(Robot robot, FullRobotModel fullRobotModel)
   {
      this.robot = robot;

      if (fullRobotModel != null)
         setFullRobotModel(fullRobotModel);
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      jointOutputWriters.clear();
      jointOutputWriters.add(new FloatingJointOutputWriter(fullRobotModel.getRootJoint(), robot.getFloatingRootJoint()));

      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList == null ? null : jointDesiredOutputList.getJointDesiredOutput(joint);
         String name = joint.getName();
         OneDoFJointBasics scsJoint = (OneDoFJointBasics) robot.findJoint(name);
         jointOutputWriters.add(new OneDoFJointOutputWriter(joint, scsJoint, jointDesiredOutput));
      }
   }
   
   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;
   }
   
   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void writeBefore(long timestamp)
   {
      //writing after
   }
   
   @Override
   public void writeAfter()
   {
      for (int i = 0; i < jointOutputWriters.size(); i++)
      {
         jointOutputWriters.get(i).write();
      }
   }

   public void updateRobotConfigurationBasedOnJointDesiredOutputPositions()
   {
      for (int i = 0; i < jointOutputWriters.size(); i++)
      {
         jointOutputWriters.get(i).updateRobotConfigurationBasedOnJointDesiredOutputPositions();
      }
   }

   public void updateRobotConfigurationBasedOnFullRobotModel()
   {
      for (int i = 0; i < jointOutputWriters.size(); i++)
      {
         jointOutputWriters.get(i).updateRobotConfigurationBasedOnFullRobotModel();
      }
   }

   private static interface JointOutputWriter
   {
      void write();

      void updateRobotConfigurationBasedOnJointDesiredOutputPositions();

      void updateRobotConfigurationBasedOnFullRobotModel();
   }

   private static class FloatingJointOutputWriter implements JointOutputWriter
   {
      private final FloatingJointBasics idJoint;
      private final SimFloatingJointBasics scsJoint;

      public FloatingJointOutputWriter(FloatingJointBasics idJoint, SimFloatingJointBasics scsJoint)
      {
         this.idJoint = idJoint;
         this.scsJoint = scsJoint;
      }

      @Override
      public void write()
      {
      }

      @Override
      public void updateRobotConfigurationBasedOnJointDesiredOutputPositions()
      {
      }

      @Override
      public void updateRobotConfigurationBasedOnFullRobotModel()
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         idJoint.getJointConfiguration(transform);
         scsJoint.setJointConfiguration(transform);
         //         scsJoint.setRotationAndTranslation(transform);
      }
   }

   private static class OneDoFJointOutputWriter implements JointOutputWriter
   {
      private final OneDoFJointBasics idJoint;
      private final OneDoFJointBasics scsJoint;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public OneDoFJointOutputWriter(OneDoFJointBasics idJoint, OneDoFJointBasics scsJoint, JointDesiredOutputReadOnly jointDesiredOutput)
      {
         this.idJoint = idJoint;
         this.scsJoint = scsJoint;
         this.jointDesiredOutput = jointDesiredOutput;
      }

      @Override
      public void write()
      {
         double tau;
         if (jointDesiredOutput != null)
            tau = jointDesiredOutput.getDesiredTorque();
         else
            tau = idJoint.getTau();

         scsJoint.setTau(tau);
      }

      @Override
      public void updateRobotConfigurationBasedOnJointDesiredOutputPositions()
      {
         double q = jointDesiredOutput.getDesiredPosition();
         scsJoint.setQ(q);

         double qd = jointDesiredOutput.getDesiredVelocity();
         scsJoint.setQd(qd);

         double qdd = jointDesiredOutput.getDesiredAcceleration();
         scsJoint.setQdd(qdd);
      }

      @Override
      public void updateRobotConfigurationBasedOnFullRobotModel()
      {
         scsJoint.setQ(idJoint.getQ());
         scsJoint.setQd(idJoint.getQd());
         scsJoint.setQdd(idJoint.getQdd());
      }
   }
}
