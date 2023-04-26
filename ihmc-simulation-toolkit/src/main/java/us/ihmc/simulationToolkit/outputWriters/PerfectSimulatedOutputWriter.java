package us.ihmc.simulationToolkit.outputWriters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class PerfectSimulatedOutputWriter implements OutputWriter
{
   private final String name;
   private final FloatingRootJointRobot robot;
   private JointDesiredOutputListReadOnly jointDesiredOutputList;
   private final List<JointOutputWriter> jointOutputWriters = new ArrayList<>();

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
      jointOutputWriters.clear();

      jointOutputWriters.add(new FloatingJointOutputWriter(fullRobotModel.getRootJoint(), robot.getRootJoint()));

      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList == null ? null : jointDesiredOutputList.getJointDesiredOutput(joint);

         if (joint instanceof CrossFourBarJoint)
         {
            CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) joint;
            OneDegreeOfFreedomJoint scsJointA = robot.getOneDegreeOfFreedomJoint(fourBarJoint.getJointA().getName());
            OneDegreeOfFreedomJoint scsJointB = robot.getOneDegreeOfFreedomJoint(fourBarJoint.getJointB().getName());
            OneDegreeOfFreedomJoint scsJointC = robot.getOneDegreeOfFreedomJoint(fourBarJoint.getJointC().getName());
            OneDegreeOfFreedomJoint scsJointD = robot.getOneDegreeOfFreedomJoint(fourBarJoint.getJointD().getName());
            jointOutputWriters.add(new CrossFourBarJointOutputWriter(fourBarJoint, scsJointA, scsJointB, scsJointC, scsJointD, jointDesiredOutput));
         }
         else
         {
            String name = joint.getName();
            OneDegreeOfFreedomJoint scsJoint = robot.getOneDegreeOfFreedomJoint(name);
            jointOutputWriters.add(new OneDoFJointOutputWriter(joint, scsJoint, jointDesiredOutput));
         }
      }
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
      private final FloatingJoint scsJoint;

      public FloatingJointOutputWriter(FloatingJointBasics idJoint, FloatingJoint scsJoint)
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
         scsJoint.setRotationAndTranslation(transform);
      }
   }

   private static class OneDoFJointOutputWriter implements JointOutputWriter
   {
      private final OneDoFJointBasics idJoint;
      private final OneDegreeOfFreedomJoint scsJoint;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public OneDoFJointOutputWriter(OneDoFJointBasics idJoint, OneDegreeOfFreedomJoint scsJoint, JointDesiredOutputReadOnly jointDesiredOutput)
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

   private static class CrossFourBarJointOutputWriter implements JointOutputWriter
   {
      private final CrossFourBarJoint idJoint;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final int[] torqueSourceIndices;
      private final OneDegreeOfFreedomJoint[] scsJoints;

      private final CrossFourBarJoint clonedIDJoint;

      public CrossFourBarJointOutputWriter(CrossFourBarJoint idJoint,
                                              OneDegreeOfFreedomJoint scsJointA,
                                              OneDegreeOfFreedomJoint scsJointB,
                                              OneDegreeOfFreedomJoint scsJointC,
                                              OneDegreeOfFreedomJoint scsJointD,
                                              JointDesiredOutputReadOnly jointDesiredOutput)
      {
         this.idJoint = idJoint;
         this.jointDesiredOutput = jointDesiredOutput;

         scsJoints = new OneDegreeOfFreedomJoint[] {scsJointA, scsJointB, scsJointC, scsJointD};
         if (idJoint.getJointA().isLoopClosure() || idJoint.getJointD().isLoopClosure())
            torqueSourceIndices = new int[] {1, 2};
         else
            torqueSourceIndices = new int[] {0, 3};

         clonedIDJoint = CrossFourBarJoint.cloneCrossFourBarJoint(idJoint, ReferenceFrameTools.constructARootFrame("fourBarClone"), "clone");
      }

      @Override
      public void write()
      {
         double tau_actuated;
         if (jointDesiredOutput != null)
            tau_actuated = idJoint.computeActuatedJointTau(jointDesiredOutput.getDesiredTorque());
         else
            tau_actuated = idJoint.getActuatedJoint().getTau();
         /*
          * Ideally we just want to set the torque of the actuated joint, but spreading the torque onto the
          * 2-joint chain that goes through the 4-bar w/o relying on the loop closure makes it a little nicer
          * on SCS's soft constraint.
          */
         // scsActuatedJoint.setTau(tau_actuated);

         for (int torqueSourceIndex : torqueSourceIndices)
         {
            double tau = 0.5 * tau_actuated / idJoint.getFourBarFunction().getLoopJacobian().get(torqueSourceIndex);
            scsJoints[torqueSourceIndex].setTau(tau);
         }
      }

      @Override
      public void updateRobotConfigurationBasedOnJointDesiredOutputPositions()
      {
         clonedIDJoint.setQ(jointDesiredOutput.getDesiredPosition());
         clonedIDJoint.setQd(jointDesiredOutput.getDesiredVelocity());
         clonedIDJoint.setQdd(jointDesiredOutput.getDesiredAcceleration());
         clonedIDJoint.updateFramesRecursively();

         for (int i = 0; i < 4; i++)
         {
            OneDegreeOfFreedomJoint scsJoint = scsJoints[i];
            if (scsJoint == null)
               continue;

            RevoluteJointBasics idSubJoint = clonedIDJoint.getFourBarFunction().getLoopJoints().get(i);
            scsJoint.setQ(idSubJoint.getQ());
            scsJoint.setQd(idSubJoint.getQd());
            scsJoint.setQdd(idSubJoint.getQdd());
         }
      }

      @Override
      public void updateRobotConfigurationBasedOnFullRobotModel()
      {
         for (int i = 0; i < 4; i++)
         {
            OneDegreeOfFreedomJoint scsJoint = scsJoints[i];
            if (scsJoint == null)
               continue;

            RevoluteJointBasics idSubJoint = idJoint.getFourBarFunction().getLoopJoints().get(i);
            scsJoint.setQ(idSubJoint.getQ());
         }
      }
   }
}
