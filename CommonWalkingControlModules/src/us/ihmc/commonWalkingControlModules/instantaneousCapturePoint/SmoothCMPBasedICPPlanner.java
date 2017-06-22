package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CenterOfPressurePlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private final ReferenceCenterOfPressureTrajectoryCalculator referenceCoPsCalculator;
   
   public SmoothCMPBasedICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                   CapturePointPlannerParameters icpPlannerParameters, CenterOfPressurePlannerParameters copPlannerParameters,
                                   YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, icpPlannerParameters);

      referenceCoPsCalculator = new ReferenceCenterOfPressureTrajectoryCalculator("icpPlanner");
      referenceCoPsCalculator.initializeParameters(copPlannerParameters, bipedSupportPolygons, contactableFeet, parentRegistry);
   }
   
   public void clearPlan()
   {
      referenceCoPsCalculator.clear();
   }
   
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      referenceCoPsCalculator.addFootstepToPlan(footstep, timing);
   }

   @Override
   public void initializeForStanding(double initialTime)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void initializeForTransfer(double initialTime)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void computeFinalCoMPositionInTransfer()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void initializeForSingleSupport(double initialTime)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void computeFinalCoMPositionInSwing()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   protected void updateTransferPlan()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   protected void updateSingleSupportPlan()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void compute(double time)
   {
      throw new RuntimeException("to implement");
   }


   @Override
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void getFinalDesiredCenterOfMassPosition(FramePoint2d finalDesiredCenterOfMassPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public boolean isOnExitCMP()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public int getNumberOfFootstepsToConsider()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   public int getNumberOfFootstepsRegistered()
   {
      throw new RuntimeException("to implement");
   }
}
