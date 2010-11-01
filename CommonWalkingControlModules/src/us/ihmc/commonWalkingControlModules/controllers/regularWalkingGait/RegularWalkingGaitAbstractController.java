package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;

import com.yobotics.simulationconstructionset.util.statemachines.State;

public class RegularWalkingGaitAbstractController
{
   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final UpperBodyTorques upperBodyTorques = new UpperBodyTorques();

   
   public RegularWalkingGaitAbstractController(
         RobotSpecificJointNames robotJointNames,
         DoEveryTickSubController doEveryTickSubController,
         StanceSubController stanceSubController,
         SwingSubController swingSubController,
         UpperBodySubController upperBodySubController
   
   )
   {
      this.robotJointNames = robotJointNames;
      
      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
      
      lowerBodyTorques = new LowerBodyTorques(robotJointNames);
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
   
}
