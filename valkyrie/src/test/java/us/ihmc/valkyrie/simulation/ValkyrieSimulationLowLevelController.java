package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieSimulationLowLevelController extends SimpleRobotController
{
   private final List<OneDoFJointController> jointControllers = new ArrayList<>();
   private final FullRobotModel controllerRobot;
   private final Robot simulatedRobot;
   private final JointDesiredOutputListReadOnly controllerDesiredOutputList;

   public ValkyrieSimulationLowLevelController(FullRobotModel controllerRobot, Robot simulatedRobot, JointDesiredOutputListReadOnly controllerDesiredOutputList,
                                               double controlDT)
   {
      this.controllerRobot = controllerRobot;
      this.simulatedRobot = simulatedRobot;
      this.controllerDesiredOutputList = controllerDesiredOutputList;
   }

   public void addJointControllers(Collection<String> jointNames)
   {
      for (String jointName : jointNames)
      {
         addJointController(jointName);
      }
   }

   public void addJointControllers(String... jointNames)
   {
      for (String jointName : jointNames)
      {
         addJointController(jointName);
      }
   }

   public void addJointController(String jointName)
   {
      OneDoFJointBasics controllerJoint = controllerRobot.getOneDoFJointByName(jointName);

      if (jointControllers.stream().anyMatch(controller -> controller.controllerJoint == controllerJoint))
         return;

      OneDegreeOfFreedomJoint simulatedJoint = (OneDegreeOfFreedomJoint) simulatedRobot.getJoint(jointName);
      JointDesiredOutputReadOnly jointDesiredOutput = controllerDesiredOutputList.getJointDesiredOutput(controllerJoint);
      jointControllers.add(new OneDoFJointController(controllerJoint, simulatedJoint, jointDesiredOutput, registry));
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl();
      }
   }

   private class OneDoFJointController
   {
      private final OneDoFJointBasics controllerJoint;
      private final OneDegreeOfFreedomJoint simulatedJoint;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoPositionTau, yoVelocityTau;

      public OneDoFJointController(OneDoFJointBasics controllerJoint, OneDegreeOfFreedomJoint simulatedJoint, JointDesiredOutputReadOnly jointDesiredOutput,
                                   YoRegistry registry)
      {
         this.controllerJoint = controllerJoint;
         this.simulatedJoint = simulatedJoint;
         this.jointDesiredOutput = jointDesiredOutput;

         String prefix = controllerJoint.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);
      }

      public void doControl()
      {
         if (jointDesiredOutput.getControlMode() != JointDesiredControlMode.POSITION)
            return;

         double positionError;
         double velocityError;

         if (jointDesiredOutput.hasDesiredPosition())
            positionError = jointDesiredOutput.getDesiredPosition() - simulatedJoint.getQ();
         else
            positionError = 0.0;

         if (jointDesiredOutput.hasDesiredVelocity())
            velocityError = jointDesiredOutput.getDesiredVelocity() - simulatedJoint.getQD();
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
         simulatedJoint.setTau(yoPositionTau.getValue() + yoVelocityTau.getValue());
      }
   }
}
