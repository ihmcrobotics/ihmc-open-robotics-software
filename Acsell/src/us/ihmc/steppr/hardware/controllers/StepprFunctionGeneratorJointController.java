package us.ihmc.steppr.hardware.controllers;

import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class StepprFunctionGeneratorJointController extends StepprPDJointController
{

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<StepprJoint> funcGenJoint = new EnumYoVariable<>("funcGenJoint", registry, StepprJoint.class);

   YoFunctionGenerator funcGen = new YoFunctionGenerator("FuncGen", registry);

   public StepprFunctionGeneratorJointController()
   {
      super();
      funcGen.setAmplitude(0);
      funcGen.setOffset(0);
      funcGen.setMode(YoFunctionGeneratorMode.OFF);
      funcGenJoint.set(StepprJoint.LEFT_KNEE_Y);
   }
   
   
   @Override
   public void doControl(long timestamp)
   {
      super.doControl(timestamp);
      for(int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         if(joint.getName().equals(funcGenJoint.getEnumValue().getSdfName()))
         {
            joint.setTau(funcGen.getValue()+joint.getTau());
         }
      } 
   }

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprFunctionGeneratorJointController());
   }

}
