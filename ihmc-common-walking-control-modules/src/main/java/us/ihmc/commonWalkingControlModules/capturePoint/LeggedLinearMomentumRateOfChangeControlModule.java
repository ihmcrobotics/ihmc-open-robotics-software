package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

public abstract class LeggedLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   protected RobotSide supportSide = null;
   protected RobotSide transferToSide = null;
   protected final YoEnum<RobotSide> supportLegPreviousTick;

   protected final YoFramePoint2d yoUnprojectedDesiredCMP;
   protected final CMPProjector cmpProjector;

   public LeggedLinearMomentumRateOfChangeControlModule(String namePrefix, ReferenceFrames referenceFrames, double gravityZ, double totalMass,
                                                       YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                       boolean use2dProjection)
   {
      super(namePrefix, referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry);

      supportLegPreviousTick = YoEnum.create(namePrefix + "SupportLegPreviousTick", "", RobotSide.class, registry, true);
      yoUnprojectedDesiredCMP = new YoFramePoint2d("unprojectedDesiredCMP", ReferenceFrame.getWorldFrame(), registry);

      if (use2dProjection)
         cmpProjector = new SmartCMPProjector(yoGraphicsListRegistry, registry);
      else
         cmpProjector = new SmartCMPPlanarProjector(registry);

      if (yoGraphicsListRegistry != null)
      {
         String graphicListName = getClass().getSimpleName();
         YoGraphicPosition unprojectedDesiredCMPViz = new YoGraphicPosition("Unprojected Desired CMP", yoUnprojectedDesiredCMP, 0.008, Purple(),
                                                                            YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
         YoArtifactPosition artifact = unprojectedDesiredCMPViz.createArtifact();
         artifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, artifact);

         //         YoArtifactPolygon yoSafeArea = new YoArtifactPolygon("SafeArea", yoSafeAreaPolygon, Color.GREEN, false);
         //         yoGraphicsListRegistry.registerArtifact(graphicListName, yoSafeArea);
         //
         //         YoArtifactPolygon yoProjectionArea = new YoArtifactPolygon("ProjectionArea", yoProjectionPolygon, Color.RED, false);
         //         yoGraphicsListRegistry.registerArtifact(graphicListName, yoProjectionArea);
      }
      yoUnprojectedDesiredCMP.setToNaN();
   }

   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         this.transferToSide = robotSide.getOppositeSide();
   }
   
   @Override
   public void compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack)
   {
      super.compute(desiredCMPPreviousValue, desiredCMPToPack);
      supportLegPreviousTick.set(supportSide);
   }
   
   public abstract void clearPlan();

   public abstract void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   public abstract void setFinalTransferDuration(double finalTransferDuration);

   public abstract void initializeForStanding();

   public abstract void initializeForSingleSupport();

   public abstract void initializeForTransfer();

   public abstract boolean getUpcomingFootstepSolution(Footstep footstepToPack);

   public abstract void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   public abstract ICPOptimizationControllerInterface getICPOptimizationController();

   public abstract void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions);
}
