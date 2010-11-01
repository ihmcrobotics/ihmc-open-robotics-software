package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;

public class RegularWalkingGaitAbstractController
{
   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final UpperBodyTorques upperBodyTorques = new UpperBodyTorques();
   protected final ProcessedOutputsInterface processedOutputs;

   protected final YoVariableRegistry controllerRegistry;
   protected final YoVariableRegistry childRegistry = new YoVariableRegistry("GoAndSuch");

   protected final BooleanYoVariable go = new BooleanYoVariable("go", "Starts and stops the walking", childRegistry);

   protected final BooleanYoVariable resetSteps = new BooleanYoVariable("resetSteps", childRegistry);
   protected final IntYoVariable stepsTaken = new IntYoVariable("stepsTaken", childRegistry);
   protected final IntYoVariable stepsToTake = new IntYoVariable("stepsToTake", childRegistry);
   protected final BooleanYoVariable onFinalStep = new BooleanYoVariable("onFinalStep", childRegistry);   
   
   protected final StateMachine walkingStateMachine;
   protected final EnumYoVariable<RobotSide> supportLegYoVariable = new EnumYoVariable<RobotSide>("supportLeg", "Current support leg. Null if double support", childRegistry, RobotSide.class);
   protected final EnumYoVariable<RobotSide> swingLegYoVariable = EnumYoVariable.create("swingLeg", "Current support leg. Null if double support", RobotSide.class, childRegistry);

   
   public RegularWalkingGaitAbstractController(
         RobotSpecificJointNames robotJointNames,
         DoubleYoVariable time,
         ProcessedOutputsInterface processedOutputs, 
         DoEveryTickSubController doEveryTickSubController,
         StanceSubController stanceSubController,
         SwingSubController swingSubController,
         UpperBodySubController upperBodySubController,
         YoVariableRegistry controllerRegistry
   )
   {
      this.robotJointNames = robotJointNames;
      this.processedOutputs = processedOutputs;      
      
      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
      
      this.controllerRegistry = controllerRegistry;
      controllerRegistry.addChild(childRegistry);
      
      lowerBodyTorques = new LowerBodyTorques(robotJointNames);
      
      walkingStateMachine = new StateMachine("walkingState", "switchTime", RegularWalkingState.class, time, childRegistry);
   }
   
   
   public enum RegularWalkingState
   {
      StartStopWalkingDoubleSupportState, UnloadRightLegToTransferIntoWalking, UnloadLeftLegToTransferIntoWalking, 
      LeftLoadingRightPreSwingA,LeftLoadingRightPreSwingB, LeftLoadingRightPreSwingC, 
      LeftEarlyStanceRightInitialSwing, LeftLateStanceRightMidSwing, LeftTerminalStanceRightTerminalSwing,
      RightLoadingLeftPreSwingA, RightLoadingLeftPreSwingB, RightLoadingLeftPreSwingC, 
      RightEarlyStanceLeftInitialSwing, RightLateStanceLeftMidSwing, RightTerminalStanceLeftTerminalSwing,
      
      StartWalkingDoubleSupportState, TransferAllLoadToRightLegForWalking,
      TransferAllLoadToLeftLegForWalking, stopWalkingLeftLoadingState, stopWalkingRightLoadingState
      ;

      public RobotSide getSupportLeg()
      {
         if ((this == LeftLoadingRightPreSwingC) || (this == LeftEarlyStanceRightInitialSwing) || (this == LeftLateStanceRightMidSwing)
                 || (this == LeftTerminalStanceRightTerminalSwing))
         {
            return RobotSide.LEFT;
         }

         else if ((this == RightLoadingLeftPreSwingC) || (this == RightEarlyStanceLeftInitialSwing) || (this == RightLateStanceLeftMidSwing)
                  || (this == RightTerminalStanceLeftTerminalSwing))
         {
            return RobotSide.RIGHT;
         }

         else if ((this == StartStopWalkingDoubleSupportState) || (this == UnloadRightLegToTransferIntoWalking) || (this == UnloadLeftLegToTransferIntoWalking)
                  || (this == RightLoadingLeftPreSwingA) || (this == RightLoadingLeftPreSwingB) || (this == LeftLoadingRightPreSwingA)
                  || (this == LeftLoadingRightPreSwingB))
         {
            return null;
         }

         else
            throw new RuntimeException("Not implemented for " + this);
      }
   }
   
   
   
   protected void setProcessedOutputsBodyTorques()
   {
      processedOutputs.setLowerBodyTorques(lowerBodyTorques);
      processedOutputs.setUpperBodyTorques(upperBodyTorques);
   }
   

   protected void setLowerBodyTorquesToZero()
   {
      lowerBodyTorques.setLowerBodyTorquesToZero();
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
   }
   
}
