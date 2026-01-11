package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePendulumControllerDefinition implements ControllerDefinition
{
   private double setpoint1;
   private double setpoint2;

   public void setSetpoints(double setpoint1, double setpoint2)
   {
      this.setpoint1 = setpoint1;
      this.setpoint2 = setpoint2;
   }

   @Override
   public Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      OneDoFJointReadOnly[] joints = MultiBodySystemTools.filterJoints(controllerOutput.getInput().getJointsToConsider(), OneDoFJointReadOnly.class)
                                                         .toArray(new OneDoFJointReadOnly[0]);
      DoublePendulumController controller = new DoublePendulumController(joints, controllerOutput.getOneDoFJointOutputs(joints));
      controller.setSetpoints(setpoint1, setpoint2);
      return controller;
   }

   private static class DoublePendulumController implements Controller
   {
      private static final double maxTau = 1000;

      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final OneDoFJointReadOnly[] jointsToControl;
      private final OneDoFJointStateBasics[] jointOutputs;

      private final YoDouble jointStiffness = new YoDouble("jointStiffness", registry);
      private final YoDouble jointDamping = new YoDouble("jointDamping", registry);

      private final YoDouble joint1Setpoint = new YoDouble("joint1Setpoint", registry);
      private final YoDouble joint2Setpoint = new YoDouble("joint2Setpoint", registry);
      private final YoDouble[] jointSetpoints = new YoDouble[] {joint1Setpoint, joint2Setpoint};

      public DoublePendulumController(OneDoFJointReadOnly[] jointsToControl, OneDoFJointStateBasics[] jointOutputs)
      {
         this.jointsToControl = jointsToControl;
         this.jointOutputs = jointOutputs;
         jointStiffness.set(350.0);
         jointDamping.set(30.0);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      public void setSetpoints(double q1, double q2)
      {
         joint1Setpoint.set(q1);
         joint2Setpoint.set(q2);
      }

      @Override
      public void doControl()
      {
         for (int i = 0; i < jointsToControl.length; i++)
         {
            double deltaQ = AngleTools.computeAngleDifferenceMinusPiToPi(jointSetpoints[i].getValue(), jointsToControl[i].getQ());
            double tau = jointStiffness.getDoubleValue() * deltaQ - jointDamping.getValue() * jointsToControl[i].getQd();
            jointOutputs[i].setEffort(MathTools.clamp(tau, maxTau));
         }
      }
   }
}
