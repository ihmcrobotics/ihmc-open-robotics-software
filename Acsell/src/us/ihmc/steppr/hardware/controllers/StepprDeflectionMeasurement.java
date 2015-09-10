package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class StepprDeflectionMeasurement implements StepprController
{

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprStandPrep");

   private final EnumMap<StepprJoint, OneDoFJoint> joints = new EnumMap<>(StepprJoint.class);

   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> desiredForces = new EnumMap<>(StepprStandPrepSetpoints.class);
   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> dampingValues = new EnumMap<>(StepprStandPrepSetpoints.class);

   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableForceOutput", registry);

   @Override
   public void setFullRobotModel(SDFFullRobotModel fullRobotModel)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         joints.put(joint, fullRobotModel.getOneDoFJointByName(joint.getSdfName()));
      }

      for (StepprStandPrepSetpoints setpoint : StepprStandPrepSetpoints.values)
      {
         DoubleYoVariable desiredForce = new DoubleYoVariable(setpoint.getName() + "_tau_d", registry);
         DoubleYoVariable damping = new DoubleYoVariable(setpoint.getName() + "_damping", registry);

         desiredForce.set(0.0);
         damping.set(0.0);

         desiredForces.put(setpoint, desiredForce);
         dampingValues.put(setpoint, damping);
      }

   }

   @Override
   public void initialize(long timestamp)
   {
      for (StepprStandPrepSetpoints setpoint : StepprStandPrepSetpoints.values)
      {
         desiredForces.get(setpoint).set(0.0);
      }
      enableOutput.set(false);
   }

   @Override
   public void doControl(long timestamp)
   {

      for (StepprStandPrepSetpoints jointGroup : StepprStandPrepSetpoints.values)
      {
         double setpoint = desiredForces.get(jointGroup).getDoubleValue();
         double damping = dampingValues.get(jointGroup).getDoubleValue();

         for (int i = 0; i < jointGroup.getJoints().length; i++)
         {
            StepprJoint joint = jointGroup.getJoints()[i];
            OneDoFJoint oneDoFJoint = joints.get(joint);
            double reflectedSetpoint = setpoint;
            if (i == 1)
            {
               if(jointGroup == StepprStandPrepSetpoints.HIP_Y)
               {
                  reflectedSetpoint *= -1;
               }
               else
               {
                  reflectedSetpoint *= jointGroup.getReflectRight();
               }
            }

            oneDoFJoint.setTau(reflectedSetpoint);
            oneDoFJoint.setKd(damping);

         }
      }

   }

   @Override
   public boolean turnOutputOn()
   {
      boolean enable = enableOutput.getBooleanValue();
      enableOutput.set(false); // Outputs only need to be enabled once, after that you can turn them off
      return enable;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprDeflectionMeasurement());
   }

}
