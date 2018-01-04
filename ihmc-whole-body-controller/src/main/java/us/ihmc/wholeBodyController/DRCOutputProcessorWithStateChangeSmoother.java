package us.ihmc.wholeBodyController;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commons.Conversions;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DRCOutputProcessorWithStateChangeSmoother implements DRCOutputProcessor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble alphaForJointTorqueForStateChanges = new YoDouble("alphaJointTorqueForStateChanges", registry);

   private final PairList<JointDesiredOutput, AlphaFilteredYoVariable> jointTorquesSmoothedAtStateChange = new PairList<>();
//   private final LinkedHashMap<LowLevelJointData, AlphaFilteredYoVariable> jointTorquesSmoothedAtStateChange = new LinkedHashMap<>();

   private final AtomicBoolean hasHighLevelControllerStateChanged = new AtomicBoolean(false);
   private final YoDouble timeAtHighLevelControllerStateChange = new YoDouble("timeAtControllerStateChange", registry);
   private final YoDouble slopTime = new YoDouble("slopTimeForSmoothedJointTorques", registry);

   private final DRCOutputProcessor drcOutputProcessor;

   public DRCOutputProcessorWithStateChangeSmoother(DRCOutputProcessor drcOutputWriter)
   {
      this.drcOutputProcessor = drcOutputWriter;
      if(drcOutputWriter != null)
      {
         registry.addChild(drcOutputWriter.getControllerYoVariableRegistry());
      }

      alphaForJointTorqueForStateChanges.set(0.0);
      slopTime.set(0.16);
      timeAtHighLevelControllerStateChange.set(Double.NEGATIVE_INFINITY);
   }

   @Override
   public void initialize()
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.initialize();
      }
   }

   @Override
   public void processAfterController(long timestamp)
   {
      if (hasHighLevelControllerStateChanged.get())
      {
         hasHighLevelControllerStateChanged.set(false);
         timeAtHighLevelControllerStateChange.set(Conversions.nanosecondsToSeconds(timestamp));
      }

      double currentTime = Conversions.nanosecondsToSeconds(timestamp);
      double deltaTime = Math.max(currentTime - timeAtHighLevelControllerStateChange.getDoubleValue(), 0.0);

      if (deltaTime < slopTime.getDoubleValue())
      {
         alphaForJointTorqueForStateChanges.set(1.0 - deltaTime / slopTime.getDoubleValue());
      }
      else
      {
         alphaForJointTorqueForStateChanges.set(0.0);
      }

      for (int i = 0; i < jointTorquesSmoothedAtStateChange.size(); i++)
      {
         JointDesiredOutput jointData = jointTorquesSmoothedAtStateChange.first(i);
         double tau = jointData.hasDesiredTorque() ? jointData.getDesiredTorque() : 0.0;
         AlphaFilteredYoVariable smoothedJointTorque = jointTorquesSmoothedAtStateChange.second(i);
         smoothedJointTorque.update(tau);
         jointData.setDesiredTorque(smoothedJointTorque.getDoubleValue());
      }

      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.processAfterController(timestamp);
      }
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

   @Override
   public void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.setLowLevelControllerCoreOutput(controllerRobotModel, lowLevelControllerCoreOutput, rawJointSensorDataHolderMap);
      }


      for (int i = 0; i < lowLevelControllerCoreOutput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         JointDesiredOutput jointData = lowLevelControllerCoreOutput.getJointDesiredOutput(i);
         String jointName = lowLevelControllerCoreOutput.getOneDoFJoint(i).getName();

         AlphaFilteredYoVariable jointTorqueSmoothedAtStateChange = new AlphaFilteredYoVariable("smoothed_tau_" + jointName, registry, alphaForJointTorqueForStateChanges);
         jointTorquesSmoothedAtStateChange.add(jointData, jointTorqueSmoothedAtStateChange);
      }
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForController);
      }
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

}
