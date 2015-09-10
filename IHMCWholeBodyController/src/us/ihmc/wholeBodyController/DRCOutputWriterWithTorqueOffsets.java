package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class DRCOutputWriterWithTorqueOffsets implements DRCOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DRCOutputWriter drcOutputWriter;
   
   private final DoubleYoVariable alphaTorqueOffset =
         new DoubleYoVariable("alphaTorqueOffset",
            "Filter for integrating acceleration to get a torque offset at each joint", registry);

   private final BooleanYoVariable resetTorqueOffsets = new BooleanYoVariable("resetTorqueOffsets", registry);
   
   private ArrayList<OneDoFJoint> oneDoFJoints;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> torqueOffsetMap;

   public DRCOutputWriterWithTorqueOffsets(DRCOutputWriter drcOutputWriter, double updateDT, boolean runningOnRealRobot)
   {
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
         if (resetTorqueOffsets.getBooleanValue()) torqueOffsetVariable.set(0.0);
         
         double offsetTorque = torqueOffsetVariable.getDoubleValue();
         double ditherTorque = 0.0;
         
         double alpha = alphaTorqueOffset.getDoubleValue();
         offsetTorque = alpha * (offsetTorque + desiredAcceleration * 0.002) + (1.0 - alpha) * offsetTorque;
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

//      for (RobotSide robotSide : RobotSide.values)
//      {
//         RigidBody shin = controllerModel.getLegJoint(robotSide, LegJointName.KNEE).getSuccessor();
//         ArrayList<OneDoFJoint> ankleJoints = new ArrayList<>(Arrays.asList(ScrewTools.filterJoints(ScrewTools.createJointPath(shin, controllerModel.getFoot(robotSide)), OneDoFJoint.class)));
//         oneDoFJoints.removeAll(ankleJoints);
//      }
      
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

   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffsetToAdd)
   {
      DoubleYoVariable torqueOffsetVariable = torqueOffsetMap.get(oneDoFJoint);
      torqueOffsetVariable.sub(torqueOffsetToAdd);
   }
}

