package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OverhauledToeOffManager
{
   private Footstep nextFootstep;
   private Footstep nextNextFootstep;

   private final ToeOffStepPositionInspector stepPositionInspector;
   private final LegJointLimitsInspector jointLimitsInspector;

   public OverhauledToeOffManager(WalkingControllerParameters walkingControllerParameters,
                                  SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                  YoRegistry parentRegistry)
   {
      ToeOffParameters toeOffParameters = walkingControllerParameters.getToeOffParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();

      double footLength = steppingParameters.getFootBackwardOffset() + steppingParameters.getFootForwardOffset();

      stepPositionInspector = new ToeOffStepPositionInspector(soleZUpFrames,
                                                              toeOffParameters,
                                                              steppingParameters.getInPlaceWidth(),
                                                              footLength,
                                                              parentRegistry);
      jointLimitsInspector = new LegJointLimitsInspector(toeOffParameters, parentRegistry);
   }


   public void submitNextFootstep(Footstep nextFootstep, Footstep nextNextFootstep)
   {
      this.nextFootstep = nextFootstep;
      this.nextNextFootstep = nextNextFootstep;
   }


}
