package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public class DRCOutputWriterWithTorqueOffsets implements DRCOutputWriter, JointTorqueOffsetProcessor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DRCOutputWriter drcOutputWriter;

   private final DoubleYoVariable alphaTorqueOffset = new DoubleYoVariable("alphaTorqueOffset",
         "Filter for integrating acceleration to get a torque offset at each joint", registry);

   private final BooleanYoVariable resetTorqueOffsets = new BooleanYoVariable("resetTorqueOffsets", registry);

   private ArrayList<OneDoFJoint> oneDoFJoints;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> torqueOffsetMap;

   private final double updateDT;

   public DRCOutputWriterWithTorqueOffsets(DRCOutputWriter drcOutputWriter, double updateDT)
   {
      this.updateDT = updateDT;
      this.drcOutputWriter = drcOutputWriter;
      registry.addChild(drcOutputWriter.getControllerYoVariableRegistry());
   }

   @Override
   public void initialize()
   {
      drcOutputWriter.initialize();
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         double desiredAcceleration = oneDoFJoint.getQddDesired();

         DoubleYoVariable torqueOffsetVariable = torqueOffsetMap.get(oneDoFJoint);
         if (resetTorqueOffsets.getBooleanValue())
            torqueOffsetVariable.set(0.0);

         double offsetTorque = torqueOffsetVariable.getDoubleValue();
         double ditherTorque = 0.0;

         double alpha = alphaTorqueOffset.getDoubleValue();
         offsetTorque = alpha * (offsetTorque + desiredAcceleration * updateDT) + (1.0 - alpha) * offsetTorque;
         torqueOffsetVariable.set(offsetTorque);
         oneDoFJoint.setTau(oneDoFJoint.getTau() + offsetTorque + ditherTorque);
      }

      drcOutputWriter.writeAfterController(timestamp);
   }

   @Override
   public void setFullRobotModel(SDFFullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      drcOutputWriter.setFullRobotModel(controllerModel, rawJointSensorDataHolderMap);

      oneDoFJoints = new ArrayList<OneDoFJoint>();
      controllerModel.getOneDoFJoints(oneDoFJoints);

      torqueOffsetMap = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         final OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         final DoubleYoVariable torqueOffset = new DoubleYoVariable("tauOffset_" + oneDoFJoint.getName(), registry);

         torqueOffsetMap.put(oneDoFJoint, torqueOffset);
      }

   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      drcOutputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForController);
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      DoubleYoVariable torqueOffsetVariable = torqueOffsetMap.get(oneDoFJoint);
      torqueOffsetVariable.sub(torqueOffset);
   }
}
