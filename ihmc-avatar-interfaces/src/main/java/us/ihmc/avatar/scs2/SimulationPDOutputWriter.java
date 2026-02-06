package us.ihmc.avatar.scs2;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
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
import us.ihmc.sensorProcessing.outputData.SimulationThreadOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Used to apply some corruption to the desired torque, velocity, and position outputs
 * from the controller and apply PD control on the torque to track them. These torques
 * then get passed on to the simulation for physics calculation.
 *
 * This class also tracks unstable velocities with some counters.
 */
public class SimulationPDOutputWriter implements SimulationThreadOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final List<JointController> jointControllers = new ArrayList<>();
   private final Map<String, JointController> jointControllerMap = new HashMap<>();

   public SimulationPDOutputWriter(ControllerInput controllerInput,
                                   ControllerOutput controllerOutput)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
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

   @Override
   public void writeBefore(long timestamp)
   {

   }

   @Override
   public void writeAfter()
   {

   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl();
      }
   }



   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private interface JointController
   {
      void doControl();
   }

   private static class OneDoFJointController implements JointController
   {
      private final OneDoFJointReadOnly simOutput;
      private final OneDoFJointStateBasics simInput;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;


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
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);

      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         if (jointDesiredOutput.hasDesiredTorque())
         {
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         }
         else
         {
            yoControllerTau.set(0.0);
         }

         if (jointDesiredOutput.hasDesiredPosition())
         {
            positionError = jointDesiredOutput.getDesiredPosition()- simOutput.getQ();
         }
         else
         {
            positionError = 0.0;
         }

         if (jointDesiredOutput.hasDesiredVelocity())
         {
            velocityError = jointDesiredOutput.getDesiredVelocity() - simOutput.getQd();
         }
         else
         {
            velocityError = 0.0;
         }

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         kp.set(jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0);
         kd.set(jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0);

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         simInput.setEffort(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
      }
   }

   private static class CrossFourBarJointController implements JointController
   {
      private final CrossFourBarJoint localFourBarJoint;
      private final OneDoFJointReadOnly[] simOutputs;
      private final int[] torqueSourceIndices;
      private final OneDoFJointStateBasics[] simInputs;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;

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

         String prefix = controllerFourBarJoint.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);
      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         updateFourBarJoint();

         if (jointDesiredOutput.hasDesiredTorque())
         {
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         }
         else
         {
            yoControllerTau.set(0.0);
         }

         if (jointDesiredOutput.hasDesiredPosition())
         {
            positionError = jointDesiredOutput.getDesiredPosition() - localFourBarJoint.getQ();
         }
         else
         {
            positionError = 0.0;
         }

         if (jointDesiredOutput.hasDesiredVelocity())
         {
            velocityError = jointDesiredOutput.getDesiredVelocity() - localFourBarJoint.getQd();
         }
         else
         {
            velocityError = 0.0;
         }

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         kp.set(jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0);
         kd.set(jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0);

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
      }

      private void updateFourBarJoint()
      {
         localFourBarJoint.setQ(simOutputs[torqueSourceIndices[0]].getQ() + simOutputs[torqueSourceIndices[1]].getQ());
         localFourBarJoint.setQd(simOutputs[torqueSourceIndices[0]].getQd() + simOutputs[torqueSourceIndices[1]].getQd());
         localFourBarJoint.updateFrame();
      }
   }
}