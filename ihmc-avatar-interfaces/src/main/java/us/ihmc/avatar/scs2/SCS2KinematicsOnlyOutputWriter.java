package us.ihmc.avatar.scs2;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is used to pass desired accelerations through for
 * kinematics only simulations.
 */
public class SCS2KinematicsOnlyOutputWriter implements JointDesiredOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final boolean writeBeforeEstimatorTick;
   private final List<JointController> jointControllers = new ArrayList<>();
   private final Map<String, JointController> jointControllerMap = new HashMap<>();
   private JointDesiredOutputBasics floatingJointDesiredOutput;
   private SimFloatingRootJoint simFloatingRootJoint;
   private RootJointDesiredConfigurationDataReadOnly outputForRootJoint;

   public SCS2KinematicsOnlyOutputWriter(ControllerInput controllerInput,
                                         ControllerOutput controllerOutput,
                                         boolean writeBeforeEstimatorTick)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
   }

   public void setOutputForRootJoint(RootJointDesiredConfigurationDataReadOnly outputForRootJoint)
   {
      this.outputForRootJoint = outputForRootJoint;

      simFloatingRootJoint = (SimFloatingRootJoint) controllerInput.getInput().getRootBody().getChildrenJoints().get(0);
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      jointControllers.clear();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly controllerJoint = jointDesiredOutputList.getOneDoFJoint(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(i);

         if (controllerJoint instanceof CrossFourBarJointBasics)
         {
            CrossFourBarJointBasics controllerFourBarJoint = (CrossFourBarJointBasics) controllerJoint;
            if (controllerOutput.getInput().findJoint(controllerFourBarJoint.getName()) != null)
            {
               OneDoFJointStateBasics simJointInput = controllerOutput.getOneDoFJointOutput(controllerJoint);
               OneDoFJointReadOnly simJointOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
               OneDoFJointController jointController = new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry);
               jointControllers.add(jointController);
               jointControllerMap.put(controllerFourBarJoint.getName(), jointController);
            }
            else
            {
               OneDoFJointStateBasics[] simInputs = new OneDoFJointStateBasics[4];
               simInputs[0] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointA());
               simInputs[1] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointB());
               simInputs[2] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointC());
               simInputs[3] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointD());
               OneDoFJointReadOnly[] simOutputs = new OneDoFJointReadOnly[4];
               simOutputs[0] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointA().getName());
               simOutputs[1] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointB().getName());
               simOutputs[2] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointC().getName());
               simOutputs[3] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointD().getName());
               CrossFourBarJointController jointController = new CrossFourBarJointController(controllerFourBarJoint,
                                                                                             simOutputs,
                                                                                             simInputs,
                                                                                             jointDesiredOutput,
                                                                                             registry);
               jointControllers.add(jointController);
               jointControllerMap.put(controllerFourBarJoint.getName(), jointController);
            }
         }
         else
         {
            OneDoFJointStateBasics simJointInput = controllerOutput.getOneDoFJointOutput(controllerJoint);
            OneDoFJointReadOnly simJointOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
            OneDoFJointController jointController = new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry);
            jointControllers.add(jointController);
            jointControllerMap.put(simJointOutput.getName(), jointController);
         }
      }
   }

   protected void write()
   {
      simFloatingRootJoint.setJointAcceleration(0, outputForRootJoint.getDesiredAcceleration());

      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl();
      }
   }

   @Override
   public void writeBefore(long timestamp)
   {
      if (writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public void writeAfter()
   {
      if (!writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private interface JointController
   {
      void doControl();
   }

   private class OneDoFJointController implements JointController
   {
      private final OneDoFJointReadOnly simOutput;
      private final OneDoFJointStateBasics simInput;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public OneDoFJointController(OneDoFJointReadOnly simOutput,
                                   OneDoFJointStateBasics simInput,
                                   JointDesiredOutputReadOnly jointDesiredOutput,
                                   YoRegistry registry)
      {
         this.simOutput = simOutput;
         this.simInput = simInput;
         this.jointDesiredOutput = jointDesiredOutput;
      }

      @Override
      public void doControl()
      {
         if (jointDesiredOutput.hasDesiredAcceleration())
            simInput.setAcceleration(jointDesiredOutput.getDesiredAcceleration());
      }
   }

   private class CrossFourBarJointController implements JointController
   {
      private final CrossFourBarJoint localFourBarJoint;
      private final OneDoFJointReadOnly[] simOutputs;
      private final int[] torqueSourceIndices;
      private final OneDoFJointStateBasics[] simInputs;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public CrossFourBarJointController(CrossFourBarJointBasics controllerFourBarJoint,
                                         OneDoFJointReadOnly[] simOutputs,
                                         OneDoFJointStateBasics[] simInputs,
                                         JointDesiredOutputReadOnly jointDesiredOutput,
                                         YoRegistry registry)
      {
         this.simOutputs = simOutputs;
         this.simInputs = simInputs;
         this.jointDesiredOutput = jointDesiredOutput;
         localFourBarJoint = CrossFourBarJoint.cloneCrossFourBarJoint(controllerFourBarJoint, ReferenceFrameTools.constructARootFrame("dummy"), "");

         if (controllerFourBarJoint.getJointA().isLoopClosure() || controllerFourBarJoint.getJointD().isLoopClosure())
            torqueSourceIndices = new int[] {1, 2};
         else
            torqueSourceIndices = new int[] {0, 3};
      }

      @Override
      public void doControl()
      {
         updateFourBarJoint();

//         if (jointDesiredOutput.hasDesiredTorque())
         double tau_actuated = localFourBarJoint.computeActuatedJointTau(jointDesiredOutput.getDesiredTorque());
         /*
          * Ideally we just want to set the torque of the actuated joint, but spreading the torque onto the
          * 2-joint chain that goes through the 4-bar w/o relying on the loop closure makes it a little nicer
          * on SCS's soft constraint.
          */

         for (OneDoFJointStateBasics simInput : simInputs)
         {
            if (simInput != null)
               simInput.setEffort(0.0);
         }

         for (int torqueSourceIndex : torqueSourceIndices)
         {
            double tau = 0.5 * tau_actuated / localFourBarJoint.getFourBarFunction().getLoopJacobian().get(torqueSourceIndex);
            simInputs[torqueSourceIndex].setEffort(tau);
         }
      }

      private void updateFourBarJoint()
      {
         localFourBarJoint.setQ(simOutputs[torqueSourceIndices[0]].getQ() + simOutputs[torqueSourceIndices[1]].getQ());
         localFourBarJoint.setQd(simOutputs[torqueSourceIndices[0]].getQd() + simOutputs[torqueSourceIndices[1]].getQd());
         localFourBarJoint.updateFrame();
      }
   }
}