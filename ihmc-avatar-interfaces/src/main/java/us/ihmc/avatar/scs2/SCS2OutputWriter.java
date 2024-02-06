package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SCS2OutputWriter implements JointDesiredOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final boolean writeBeforeEstimatorTick;
   private final List<JointController> jointControllers = new ArrayList<>();

   private final YoDouble unstableVelocityThreshold = new YoDouble("unstableVelocityThreshold", registry);
   private final YoInteger unstableVelocityNumberThreshold = new YoInteger("unstableVelocityNumberThreshold", registry);
   private final YoDouble unstableVelocityLowDampingScale = new YoDouble("unstableVelocityLowDampingScale", registry);
   private final YoDouble unstableVelocityLowDampingDuration = new YoDouble("unstableVelocityLowDampingDuration", registry);
   private JointDesiredOutputWriter customWriter;

   public SCS2OutputWriter(ControllerInput controllerInput, ControllerOutput controllerOutput, boolean writeBeforeEstimatorTick)
   {
      this(controllerInput, controllerOutput, writeBeforeEstimatorTick, null);
   }

   public SCS2OutputWriter(ControllerInput controllerInput,
                           ControllerOutput controllerOutput,
                           boolean writeBeforeEstimatorTick,
                           JointDesiredOutputWriter customWriter)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
      this.customWriter = customWriter;

      if (customWriter != null)
         registry.addChild(customWriter.getYoVariableRegistry());

      unstableVelocityThreshold.set(0.45);
      unstableVelocityNumberThreshold.set(10);
      unstableVelocityLowDampingScale.set(0.25);
      unstableVelocityLowDampingDuration.set(0.5);
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      if (customWriter != null)
         customWriter.setJointDesiredOutputList(jointDesiredOutputList);

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
               jointControllers.add(new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry));
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
               jointControllers.add(new CrossFourBarJointController(controllerFourBarJoint, simOutputs, simInputs, jointDesiredOutput, registry));
            }
         }
         else
         {
            OneDoFJointStateBasics simJointInput = controllerOutput.getOneDoFJointOutput(controllerJoint);
            OneDoFJointReadOnly simJointOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
            jointControllers.add(new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry));
         }
      }
   }

   protected void write()
   {
      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl();
      }
   }

   @Override
   public void writeBefore(long timestamp)
   {
      if (customWriter != null)
         customWriter.writeBefore(timestamp);
      if (writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public void writeAfter()
   {
      if (customWriter != null)
         customWriter.writeAfter();
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

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;

      private final YoInteger unstableVelocityCounter;
      private final YoDouble previousVelocity;
      private final YoDouble unstableVelocityStartTime;

      public OneDoFJointController(OneDoFJointReadOnly simOutput,
                                   OneDoFJointStateBasics simInput,
                                   JointDesiredOutputReadOnly jointDesiredOutput,
                                   YoRegistry registry)
      {
         this.simOutput = simOutput;
         this.simInput = simInput;
         this.jointDesiredOutput = jointDesiredOutput;

         String prefix = simOutput.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);

         kp.set(100.0);
         kd.set(6.0);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);

         unstableVelocityCounter = new YoInteger(prefix + "UnstableVelocityCounter", registry);
         previousVelocity = new YoDouble(prefix + "PreviousVelocity", registry);
         unstableVelocityStartTime = new YoDouble(prefix + "UnstableVelocityStartTime", registry);
      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         if (jointDesiredOutput.hasDesiredTorque())
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         else
            yoControllerTau.set(0.0);

         if (jointDesiredOutput.hasDesiredPosition())
            positionError = jointDesiredOutput.getDesiredPosition() - simOutput.getQ();
         else
            positionError = 0.0;

         if (jointDesiredOutput.hasDesiredVelocity())
            velocityError = jointDesiredOutput.getDesiredVelocity() - simOutput.getQd();
         else
            velocityError = 0.0;

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         //kp.set(jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0);
         //kd.set(jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0);

         updateUnstableVelocityCounter();
         double time = controllerInput.getTime();
         if (unstableVelocityCounter.getValue() >= unstableVelocityNumberThreshold.getValue())
            unstableVelocityStartTime.set(time);

         if (time - unstableVelocityStartTime.getValue() <= unstableVelocityLowDampingDuration.getValue())
         {
            double alpha = MathTools.clamp((time - unstableVelocityStartTime.getValue()) / unstableVelocityLowDampingDuration.getValue(), 0.0, 1.0);
            kd.mul(EuclidCoreTools.interpolate(unstableVelocityLowDampingScale.getValue(), 1.0, alpha));
         }

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         simInput.setEffort(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
         previousVelocity.set(simOutput.getQd());
      }

      private void updateUnstableVelocityCounter()
      {
         boolean unstable = simOutput.getQd() * previousVelocity.getValue() < 0.0;

         if (unstable)
            unstable = !EuclidCoreTools.epsilonEquals(simOutput.getQd(), previousVelocity.getValue(), unstableVelocityThreshold.getValue());

         if (unstable)
            unstableVelocityCounter.set(Math.min(unstableVelocityCounter.getValue() + 1, unstableVelocityNumberThreshold.getValue()));
         else
            unstableVelocityCounter.set(Math.max(unstableVelocityCounter.getValue() - 1, 0));
      }
   }

   private class CrossFourBarJointController implements JointController
   {
      private final CrossFourBarJoint localFourBarJoint;
      private final OneDoFJointReadOnly[] simOutputs;
      private final int[] torqueSourceIndices;
      private final OneDoFJointStateBasics[] simInputs;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;

      private final YoInteger unstableVelocityCounter;
      private final YoDouble previousVelocity;
      private final YoDouble unstableVelocityStartTime;

      public CrossFourBarJointController(CrossFourBarJointBasics controllerFourBarJoint,
                                         OneDoFJointReadOnly[] simOutputs,
                                         OneDoFJointStateBasics[] simInputs,
                                         JointDesiredOutputReadOnly jointDesiredOutput,
                                         YoRegistry registry)
      {
         this.simOutputs = simOutputs;
         this.simInputs = simInputs;
         this.jointDesiredOutput = jointDesiredOutput;
         localFourBarJoint = CrossFourBarJoint.cloneCrossFourBarJoint(controllerFourBarJoint, ReferenceFrameTools.constructARootFrame("dummy"), "dummy");

         if (controllerFourBarJoint.getJointA().isLoopClosure() || controllerFourBarJoint.getJointD().isLoopClosure())
            torqueSourceIndices = new int[] {1, 2};
         else
            torqueSourceIndices = new int[] {0, 3};

         String prefix = controllerFourBarJoint.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);

         unstableVelocityCounter = new YoInteger(prefix + "UnstableVelocityCounter", registry);
         previousVelocity = new YoDouble(prefix + "PreviousVelocity", registry);
         unstableVelocityStartTime = new YoDouble(prefix + "UnstableVelocityStartTime", registry);
      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         updateFourBarJoint();

         if (jointDesiredOutput.hasDesiredTorque())
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         else
            yoControllerTau.set(0.0);

         if (jointDesiredOutput.hasDesiredPosition())
            positionError = jointDesiredOutput.getDesiredPosition() - localFourBarJoint.getQ();
         else
            positionError = 0.0;

         if (jointDesiredOutput.hasDesiredVelocity())
            velocityError = jointDesiredOutput.getDesiredVelocity() - localFourBarJoint.getQd();
         else
            velocityError = 0.0;

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         kp.set(jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0);
         kd.set(jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0);

         updateUnstableVelocityCounter();
         double time = controllerInput.getTime();
         if (unstableVelocityCounter.getValue() >= unstableVelocityNumberThreshold.getValue())
            unstableVelocityStartTime.set(time);

         if (time - unstableVelocityStartTime.getValue() <= unstableVelocityLowDampingDuration.getValue())
         {
            double alpha = MathTools.clamp((time - unstableVelocityStartTime.getValue()) / unstableVelocityLowDampingDuration.getValue(), 0.0, 1.0);
            kd.mul(EuclidCoreTools.interpolate(unstableVelocityLowDampingScale.getValue(), 1.0, alpha));
         }

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         double tau_actuated = localFourBarJoint.computeActuatedJointTau(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
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

         previousVelocity.set(localFourBarJoint.getQd());
      }

      private void updateFourBarJoint()
      {
         localFourBarJoint.setQ(simOutputs[torqueSourceIndices[0]].getQ() + simOutputs[torqueSourceIndices[1]].getQ());
         localFourBarJoint.setQd(simOutputs[torqueSourceIndices[0]].getQd() + simOutputs[torqueSourceIndices[1]].getQd());
         localFourBarJoint.updateFrame();
      }

      private void updateUnstableVelocityCounter()
      {
         boolean unstable = localFourBarJoint.getQd() * previousVelocity.getValue() < 0.0;

         if (unstable)
            unstable = !EuclidCoreTools.epsilonEquals(localFourBarJoint.getQd(), previousVelocity.getValue(), unstableVelocityThreshold.getValue());

         if (unstable)
            unstableVelocityCounter.set(Math.min(unstableVelocityCounter.getValue() + 1, unstableVelocityNumberThreshold.getValue()));
         else
            unstableVelocityCounter.set(Math.max(unstableVelocityCounter.getValue() - 1, 0));
      }
   }
}