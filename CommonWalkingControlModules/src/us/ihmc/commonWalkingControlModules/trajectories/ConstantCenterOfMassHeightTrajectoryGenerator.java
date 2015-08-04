package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;


public class ConstantCenterOfMassHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;

   public ConstantCenterOfMassHeightTrajectoryGenerator(double initialDesiredCoMHeight, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry);
      parentRegistry.addChild(registry);

      desiredCenterOfMassHeight.set(initialDesiredCoMHeight);
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep,
         List<PlaneContactState> contactStates)
   {
      // empty
   }

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData)
   {
      coMHeightPartialDerivativesDataToPack.setCoMHeight(ReferenceFrame.getWorldFrame(), desiredCenterOfMassHeight.getDoubleValue());
      coMHeightPartialDerivativesDataToPack.setPartialD2zDx2(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDxDy(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDy2(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialDzDx(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialDzDy(0.0);
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      return false;
   }

   public void attachWalkOnToesManager(WalkOnTheEdgesManager walkOnTheEdgesManager)
   {
   }

   @Override
   public void setSupportLeg(RobotSide supportLeg)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      // TODO Auto-generated method stub
      
   }
}