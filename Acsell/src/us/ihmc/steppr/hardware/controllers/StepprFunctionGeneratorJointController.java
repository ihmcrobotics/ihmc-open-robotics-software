package us.ihmc.steppr.hardware.controllers;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.steppr.hardware.StepprJoint;

public class StepprFunctionGeneratorJointController extends StepprPDJointController
{

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
   public void initialize(long timestamp)
   {
      
      //Initialize controller gains in super-class from StandPrep
      for(StepprStandPrepSetpoints jointPair: StepprStandPrepSetpoints.values)
      {
         for(StepprJoint stepprJoint:jointPair.getJoints())
         {                       
            for(int i=0;i<joints.size();i++)
            {
               if(joints.get(i).getName().equals(stepprJoint.getSdfName()))
               {
                  PDController controller = controllers.get(i);
                  controller.setDerivativeGain(jointPair.getKd());
                  controller.setProportionalGain(jointPair.getKp());                  
                  damping.get(i).set(jointPair.getDamping());
                  break;
               }
            }
         }
      }
         
   }

   
   @Override
   public void doControl(long timestamp)
   {
      super.doControl(timestamp);
      
      //add additional tau
      for(int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         if(joint.getName().equals(funcGenJoint.getEnumValue().getSdfName()))
         {
            joint.setTau(funcGen.getValue(TimeTools.nanoSecondstoSeconds(timestamp))+joint.getTau());
         }
      } 
   }

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprFunctionGeneratorJointController());
   }

}
