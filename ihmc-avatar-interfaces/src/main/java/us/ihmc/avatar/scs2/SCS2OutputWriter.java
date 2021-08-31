package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
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

public class SCS2OutputWriter implements JointDesiredOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final boolean writeBeforeEstimatorTick;
   private final List<JointController> jointControllers = new ArrayList<>();

   public SCS2OutputWriter(ControllerInput controllerInput, ControllerOutput controllerOutput, boolean writeBeforeEstimatorTick)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      jointControllers.clear();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly controllerJoint = jointDesiredOutputList.getOneDoFJoint(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(i);
         OneDoFJointStateBasics simJointOutput = controllerOutput.getOneDoFJointOutput(controllerJoint);
         OneDoFJointReadOnly simJointInput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
         jointControllers.add(new OneDoFJointController(simJointInput, simJointOutput, jointDesiredOutput, registry));

      }
   }

   private void write()
   {
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

      OneDoFJointReadOnly getControllerJoint();
   }

   private class OneDoFJointController implements JointController
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
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         else
            yoControllerTau.set(0.0);

         if (jointDesiredOutput.hasDesiredPosition())
            positionError = jointDesiredOutput.getDesiredPosition() - simOutput.getQ();
         else
            positionError = 0.0;

         if (jointDesiredOutput.hasDesiredVelocity())
            velocityError = jointDesiredOutput.getDesiredVelocity() - simOutput.getQ();
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

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         simInput.setEffort(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
      }

      @Override
      public OneDoFJointReadOnly getControllerJoint()
      {
         return simOutput;
      }
   }
}
