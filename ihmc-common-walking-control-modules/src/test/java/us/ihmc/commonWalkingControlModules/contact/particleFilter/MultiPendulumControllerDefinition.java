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

public class MultiPendulumControllerDefinition implements ControllerDefinition
{
   private double[] setpoints;

   public void setSetpoints(double... setpoints)
   {
      this.setpoints = setpoints;
   }

   @Override
   public Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      OneDoFJointReadOnly[] joints = MultiBodySystemTools.filterJoints(controllerOutput.getInput().getJointsToConsider(), OneDoFJointReadOnly.class)
                                                         .toArray(new OneDoFJointReadOnly[0]);
      MultiPendulumController controller = new MultiPendulumController(joints, controllerOutput.getOneDoFJointOutputs(joints));
      controller.setSetpoints(setpoints);
      return controller;
   }

   private static class MultiPendulumController implements Controller
   {
      private static final double maxTau = 1000;

      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final OneDoFJointReadOnly[] jointsToControl;
      private final OneDoFJointStateBasics[] jointOutputs;

      private final YoDouble jointStiffness = new YoDouble("jointStiffness", registry);
      private final YoDouble jointDamping = new YoDouble("jointDamping", registry);

      private final YoDouble[] jointSetpoints;

      public MultiPendulumController(OneDoFJointReadOnly[] jointsToControl, OneDoFJointStateBasics[] jointOutputs)
      {
         this.jointsToControl = jointsToControl;
         this.jointOutputs = jointOutputs;
         jointSetpoints = new YoDouble[jointsToControl.length];

         jointStiffness.set(450.0);
         jointDamping.set(40.0);

         for (int i = 0; i < jointsToControl.length; i++)
         {
            jointSetpoints[i] = new YoDouble("joint" + i + "Setpoint", registry);
         }
      }

      @Override
      public void doControl()
      {
         for (int i = 0; i < jointsToControl.length; i++)
         {
            double deltaQ = AngleTools.computeAngleDifferenceMinusPiToPi(jointSetpoints[i].getValue(), jointsToControl[i].getQ());
            double tau = jointStiffness.getDoubleValue() * deltaQ - jointDamping.getValue() * (jointsToControl[i].getQd());
            jointOutputs[i].setEffort(MathTools.clamp(tau, maxTau));
         }
      }

      public void setSetpoints(double... q)
      {
         for (int i = 0; i < Math.min(jointsToControl.length, q.length); i++)
         {
            jointSetpoints[i].set(q[i]);
         }
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }
}
