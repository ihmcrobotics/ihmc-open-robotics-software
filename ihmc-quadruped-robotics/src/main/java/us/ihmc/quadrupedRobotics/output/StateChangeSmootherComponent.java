package us.ihmc.quadrupedRobotics.output;

import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicBoolean;

public class StateChangeSmootherComponent implements OutputProcessorComponent
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleParameter slopTimeParameter = new DoubleParameter("stateChangeSmootherSlopTime", registry, 0.04);
   private final DoubleParameter slopBreakFrequencyParameter = new DoubleParameter("stateChangeSmootherSlopBreakFrequency", registry, 30.0);

   private final PairList<JointDesiredOutput, AlphaFilteredYoVariable> jointTorquesSmoothedAtStateChange = new PairList<>();
   private final YoDouble alphaJointTorqueForStateChanges = new YoDouble("alphaJointTorqueForStateChanges", registry);

   private final AtomicBoolean hasHighLevelControllerStateChanged = new AtomicBoolean(false);
   private final YoDouble timeAtHighLevelControllerStateChange = new YoDouble("timeAtControllerStateChange", registry);
   private final double controlDT;
   private final YoDouble controlTimestamp;
   private final JointDesiredOutputList jointDesiredOutputList;

   public StateChangeSmootherComponent(QuadrupedRuntimeEnvironment runtimeEnvironment, YoVariableRegistry parentRegistry)
   {
      this.controlDT = runtimeEnvironment.getControlDT();
      this.controlTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

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
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = oneDoFJoint.getName();

         AlphaFilteredYoVariable jointTorqueSmoothedAtStateChange = new AlphaFilteredYoVariable("smoothed_tau_" + jointName, registry,
               alphaJointTorqueForStateChanges);
         jointTorquesSmoothedAtStateChange.add(jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint), jointTorqueSmoothedAtStateChange);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void update()
   {
      if (hasHighLevelControllerStateChanged.getAndSet(false))
      {
         timeAtHighLevelControllerStateChange.set(controlTimestamp.getDoubleValue());
      }

      double currentTime = controlTimestamp.getDoubleValue();
      double deltaTime = Math.max(currentTime - timeAtHighLevelControllerStateChange.getDoubleValue(), 0.0);

      if (deltaTime < slopTimeParameter.getValue())
      {
         double breakFrequencyInHz = slopBreakFrequencyParameter.getValue() * (deltaTime / slopTimeParameter.getValue());
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyInHz, controlDT);
         alphaJointTorqueForStateChanges.set(alpha);
      }
      else
      {
         alphaJointTorqueForStateChanges.set(0.0);
      }

      for (int i = 0; i < jointTorquesSmoothedAtStateChange.size(); i++)
      {
         JointDesiredOutput jointData = jointTorquesSmoothedAtStateChange.first(i);
         double tau = jointData.hasDesiredTorque() ? jointData.getDesiredTorque() : 0.0;
         AlphaFilteredYoVariable smoothedJointTorque = jointTorquesSmoothedAtStateChange.second(i);
         smoothedJointTorque.update(tau);
         jointData.setDesiredTorque(smoothedJointTorque.getDoubleValue());
      }
   }

   public <K extends Enum<K>> StateChangedListener<K> createFiniteStateMachineStateChangedListener()
   {
      return (from, to) -> hasHighLevelControllerStateChanged.set(true);
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
