package us.ihmc.wholeBodyController;

import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DRCOutputProcessorWithTorqueOffsets implements DRCOutputProcessor, JointTorqueOffsetProcessor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DRCOutputProcessor drcOutputWriter;

   private final YoDouble alphaTorqueOffset = new YoDouble("alphaTorqueOffset", "Filter for integrating acceleration to get a torque offset at each joint",
                                                           registry);

   private final YoBoolean resetTorqueOffsets = new YoBoolean("resetTorqueOffsets", registry);

   private PairList<JointDesiredOutputBasics, YoDouble> torqueOffsetList;
   private HashMap<OneDoFJointBasics, YoDouble> torqueOffsetMap;

   private final double updateDT;

   public DRCOutputProcessorWithTorqueOffsets(DRCOutputProcessor drcOutputWriter, double updateDT)
   {
      this.updateDT = updateDT;
      this.drcOutputWriter = drcOutputWriter;
      if (drcOutputWriter != null)
      {
         registry.addChild(drcOutputWriter.getControllerYoVariableRegistry());
      }
   }

   @Override
   public void initialize()
   {
      if (drcOutputWriter != null)
      {
         drcOutputWriter.initialize();
      }
   }

   @Override
   public void processAfterController(long timestamp)
   {
      for (int i = 0; i < torqueOffsetList.size(); i++)
      {
         JointDesiredOutputBasics jointData = torqueOffsetList.first(i);
         YoDouble torqueOffsetVariable = torqueOffsetList.second(i);

         double desiredAcceleration = jointData.hasDesiredAcceleration() ? jointData.getDesiredAcceleration() : 0.0;

         if (resetTorqueOffsets.getBooleanValue())
            torqueOffsetVariable.set(0.0);

         double offsetTorque = torqueOffsetVariable.getDoubleValue();
         double ditherTorque = 0.0;

         double alpha = alphaTorqueOffset.getDoubleValue();
         offsetTorque = alpha * (offsetTorque + desiredAcceleration * updateDT) + (1.0 - alpha) * offsetTorque;
         torqueOffsetVariable.set(offsetTorque);
         double desiredTorque = jointData.hasDesiredTorque() ? jointData.getDesiredTorque() : 0.0;
         jointData.setDesiredTorque(desiredTorque + offsetTorque + ditherTorque);
      }

      if (drcOutputWriter != null)
      {
         drcOutputWriter.processAfterController(timestamp);
      }
   }

   @Override
   public void setLowLevelControllerCoreOutput(HumanoidRobotContextJointData estimatedState, JointDesiredOutputListBasics lowLevelControllerCoreOutput)
   {
      if (drcOutputWriter != null)
      {
         drcOutputWriter.setLowLevelControllerCoreOutput(estimatedState, lowLevelControllerCoreOutput);
      }

      torqueOffsetList = new PairList<>();
      torqueOffsetMap = new HashMap<>();

      for (int i = 0; i < lowLevelControllerCoreOutput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         JointDesiredOutputBasics jointData = lowLevelControllerCoreOutput.getJointDesiredOutput(i);
         final YoDouble torqueOffset = new YoDouble("tauOffset_" + lowLevelControllerCoreOutput.getOneDoFJoint(i).getName(), registry);

         torqueOffsetList.add(jointData, torqueOffset);
         torqueOffsetMap.put(lowLevelControllerCoreOutput.getOneDoFJoint(i), torqueOffset);
      }

   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      if (drcOutputWriter != null)
      {
         drcOutputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForController);
      }
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void subtractTorqueOffset(OneDoFJointBasics oneDoFJoint, double torqueOffset)
   {
      YoDouble torqueOffsetVariable = torqueOffsetMap.get(oneDoFJoint);
      torqueOffsetVariable.sub(torqueOffset);
   }
}
