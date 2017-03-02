package us.ihmc.quadrupedRobotics.output;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class StateChangeSmootherComponent implements OutputProcessorComponent
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter slopTimeParameter = parameterFactory.createDouble("stateChangeSmootherSlopTime", 0.0);
   private final DoubleParameter slopBreakFrequencyParameter = parameterFactory.createDouble("stateChangeSmootherSlopBreakFrequency", 1000.0);

   private final ArrayList<OneDoFJoint> allJoints = new ArrayList<>();
   private final LinkedHashMap<OneDoFJoint, AlphaFilteredYoVariable> jointTorquesSmoothedAtStateChange = new LinkedHashMap<>();
   private final DoubleYoVariable alphaJointTorqueForStateChanges = new DoubleYoVariable("alphaJointTorqueForStateChanges", registry);

   private final AtomicBoolean hasHighLevelControllerStateChanged = new AtomicBoolean(false);
   private final DoubleYoVariable timeAtHighLevelControllerStateChange = new DoubleYoVariable("timeAtControllerStateChange", registry);
   private final double controlDT;
   private final DoubleYoVariable controlTimestamp;

   public StateChangeSmootherComponent(double controlDT, DoubleYoVariable controlTimestamp, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.controlTimestamp = controlTimestamp;

      alphaJointTorqueForStateChanges.set(0.0);
      timeAtHighLevelControllerStateChange.set(Double.NEGATIVE_INFINITY);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint oneDoFJoint = joints[i];
         String jointName = oneDoFJoint.getName();
         allJoints.add(oneDoFJoint);

         AlphaFilteredYoVariable jointTorqueSmoothedAtStateChange = new AlphaFilteredYoVariable("smoothed_tau_" + jointName, registry,
               alphaJointTorqueForStateChanges);
         jointTorquesSmoothedAtStateChange.put(oneDoFJoint, jointTorqueSmoothedAtStateChange);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void update()
   {
      if (hasHighLevelControllerStateChanged.get())
      {
         hasHighLevelControllerStateChanged.set(false);
         timeAtHighLevelControllerStateChange.set(controlTimestamp.getDoubleValue());
      }

      double currentTime = controlTimestamp.getDoubleValue();
      double deltaTime = Math.max(currentTime - timeAtHighLevelControllerStateChange.getDoubleValue(), 0.0);

      if (deltaTime < slopTimeParameter.get())
      {
         double breakFrequencyInHz = slopBreakFrequencyParameter.get() * (deltaTime / slopTimeParameter.get());
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyInHz, controlDT);
         alphaJointTorqueForStateChanges.set(alpha);
      }
      else
      {
         alphaJointTorqueForStateChanges.set(0.0);
      }

      for (int i = 0; i < allJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = allJoints.get(i);
         double tau = oneDoFJoint.getTau();
         AlphaFilteredYoVariable smoothedJointTorque = jointTorquesSmoothedAtStateChange.get(oneDoFJoint);
         smoothedJointTorque.update(tau);
         oneDoFJoint.setTau(smoothedJointTorque.getDoubleValue());
      }
   }

   public FiniteStateMachineStateChangedListener createFiniteStateMachineStateChangedListener()
   {
      FiniteStateMachineStateChangedListener finiteStateMachineStateChangedListener = new FiniteStateMachineStateChangedListener()
      {
         @Override
         public void stateHasChanged(Enum<?> oldState, Enum<?> newState)
         {
            hasHighLevelControllerStateChanged.set(true);
         }
      };

      return finiteStateMachineStateChangedListener;
   }

   public ControllerStateChangedListener createControllerStateChangedListener()
   {
      ControllerStateChangedListener controllerStateChangedListener = new ControllerStateChangedListener()
      {
         @Override
         public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
         {
            hasHighLevelControllerStateChanged.set(true);
         }
      };

      return controllerStateChangedListener;
   }
}
