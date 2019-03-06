package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ICPOptimizationLinearMomentumRateOfChangeControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private Vector3DReadOnly defaultLinearMomentumRateWeight;
   private Vector3DReadOnly defaultAngularMomentumRateWeight;
   private Vector3DReadOnly highLinearMomentumRateWeight;
   private final YoFrameVector3D angularMomentumRateWeight;
   private final YoFrameVector3D linearMomentumRateWeight;

   private final YoBoolean minimizeAngularMomentumRateZ;

   private final YoFrameVector3D controlledCoMAcceleration;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private double omega0 = 0.0;
   private double totalMass;
   private double gravityZ;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMass;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final FrameVector2D capturePointVelocity = new FrameVector2D();
   private final FramePoint2D desiredCapturePoint = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();
   private final FramePoint2D finalDesiredCapturePoint = new FramePoint2D();

   private final FramePoint2D perfectCMP = new FramePoint2D();
   private final FramePoint2D perfectCoP = new FramePoint2D();
   private final FramePoint2D desiredCMP = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();

   private boolean controlHeightWithMomentum;

   private final YoFramePoint2D yoUnprojectedDesiredCMP;

   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();
   private double desiredCoMHeightAcceleration = 0.0;

   private final FramePoint3D cmp3d = new FramePoint3D();
   private final FrameVector3D groundReactionForce = new FrameVector3D();

   private boolean desiredCMPcontainedNaN = false;
   private boolean desiredCoPcontainedNaN = false;

   private final ICPOptimizationControllerInterface icpOptimizationController;

   private final YoDouble yoTime;

   private final FrameVector2D perfectCMPDelta = new FrameVector2D();

   private RobotSide supportSide = null;

   private RobotSide transferToSide = null;

   private final YoEnum<RobotSide> supportLegPreviousTick;

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
                                                                 ICPControlPolygons icpControlPolygons, SideDependentList<ContactableFoot> contactableFeet,
                                                                 WalkingControllerParameters walkingControllerParameters, YoDouble yoTime, double totalMass,
                                                                 double gravityZ, double controlDT, YoVariableRegistry parentRegistry,
                                                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.totalMass = totalMass;
      this.gravityZ = gravityZ;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMass = new FramePoint3D(centerOfMassFrame);

      controlledCoMAcceleration = new YoFrameVector3D("ControlledCoMAcceleration", "", centerOfMassFrame, registry);

      angularMomentumRateWeight = new YoFrameVector3D("AngularMomentumRateWeight", worldFrame, registry);
      linearMomentumRateWeight = new YoFrameVector3D("LinearMomentumRateWeight", worldFrame, registry);

      minimizeAngularMomentumRateZ = new YoBoolean("MinimizeAngularMomentumRateZ", registry);

      momentumRateCommand.setWeights(0.0, 0.0, 0.0, linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());

      perfectCoP.setToNaN();

      yoUnprojectedDesiredCMP = new YoFramePoint2D("unprojectedDesiredCMP", ReferenceFrame.getWorldFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         String graphicListName = getClass().getSimpleName();
         YoGraphicPosition unprojectedDesiredCMPViz = new YoGraphicPosition("Unprojected Desired CMP", yoUnprojectedDesiredCMP, 0.008, Purple(),
                                                                            YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
         YoArtifactPosition artifact = unprojectedDesiredCMPViz.createArtifact();
         artifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, artifact);
      }
      yoUnprojectedDesiredCMP.setToNaN();

      this.yoTime = yoTime;

      icpOptimizationController = new ICPOptimizationController(walkingControllerParameters, bipedSupportPolygons, icpControlPolygons, contactableFeet,
                                                                controlDT, registry, yoGraphicsListRegistry);

      supportLegPreviousTick = YoEnum.create("SupportLegPreviousTick", "", RobotSide.class, registry, true);

      parentRegistry.addChild(registry);
   }

   public void setMomentumWeight(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      defaultLinearMomentumRateWeight = linearWeight;
      defaultAngularMomentumRateWeight = angularWeight;
   }

   public void setHighMomentumWeightForRecovery(Vector3DReadOnly highLinearWeight)
   {
      highLinearMomentumRateWeight = highLinearWeight;
   }

   public void setOmega0(double omega0)
   {
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      this.omega0 = omega0;
   }

   /**
    * Sets the capture point position and velocity to be used for the next call of
    * {@link #compute(FramePoint2DReadOnly, FramePoint2D)}.
    *
    * @param capturePoint the measured position of the capture point. Not modified.
    * @param capturePointVelocity the measured velocity of the capture point. If
    *           {@code capturePointVelocity == null}, the internal reference is then set to
    *           {@link Double#NaN}. Not modified.
    */
   public void setCapturePoint(FramePoint2DReadOnly capturePoint, FrameVector2DReadOnly capturePointVelocity)
   {
      this.capturePoint.setIncludingFrame(capturePoint);
      this.capturePointVelocity.setIncludingFrame(capturePointVelocity);
   }

   public void setDesiredCapturePoint(FramePoint2DReadOnly desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public void setDesiredCapturePointVelocity(FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public void setHighMomentumWeight()
   {
      linearMomentumRateWeight.set(highLinearMomentumRateWeight);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
   }

   public void setDefaultMomentumWeight()
   {
      linearMomentumRateWeight.set(defaultLinearMomentumRateWeight);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
   }

   public void setDesiredCenterOfMassHeightAcceleration(double desiredCenterOfMassHeightAcceleration)
   {
      desiredCoMHeightAcceleration = desiredCenterOfMassHeightAcceleration;
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public void computeAchievedCMP(FrameVector3DReadOnly achievedLinearMomentumRate, FramePoint2D achievedCMPToPack)
   {
      if (achievedLinearMomentumRate.containsNaN())
         return;

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / totalMass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMPToPack.set(achievedCoMAcceleration2d);
      achievedCMPToPack.scale(-1.0 / (omega0 * omega0));
      achievedCMPToPack.add(centerOfMass2d);
   }

   private FrameVector3D computeGroundReactionForce(FramePoint2DReadOnly cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   public boolean compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack, FramePoint2D desiredCoPToPack)
   {
      boolean inputsAreOk = checkInputs(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMP);
      computeCMPInternal(desiredCMPPreviousValue);

      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);
      desiredCoP.changeFrame(worldFrame);

      if (desiredCMP.containsNaN())
      {
         if (!desiredCMPcontainedNaN)
            LogTools.error("Desired CMP contains NaN, setting it to the ICP - only showing this error once");
         desiredCMP.set(capturePoint);
         desiredCMPcontainedNaN = true;
      }
      else
      {
         desiredCMPcontainedNaN = false;
      }

      if (desiredCoP.containsNaN())
      {
         if (!desiredCoPcontainedNaN)
            LogTools.error("Desired CoP contains NaN, setting it to the desiredCMP - only showing this error once");
         desiredCoP.set(desiredCMP);
         desiredCoPcontainedNaN = true;
      }
      else
      {
         desiredCoPcontainedNaN = false;
      }

      desiredCMPToPack.setIncludingFrame(desiredCMP);
      desiredCMPToPack.changeFrame(worldFrame);

      desiredCoPToPack.setIncludingFrame(desiredCoP);
      desiredCoPToPack.changeFrame(worldFrame);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);
      FrameVector3D linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);

      linearMomentumRateOfChange.changeFrame(worldFrame);
      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);

      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearZ(controlHeightWithMomentum);
      selectionMatrix.selectAngularZ(minimizeAngularMomentumRateZ.getBooleanValue());
      momentumRateCommand.setSelectionMatrix(selectionMatrix);

      momentumRateCommand.setWeights(angularMomentumRateWeight.getX(), angularMomentumRateWeight.getY(), angularMomentumRateWeight.getZ(),
                                     linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());

      supportLegPreviousTick.set(supportSide);

      return inputsAreOk;
   }

   private static boolean checkInputs(FramePoint2DReadOnly capturePoint, FramePoint2DBasics desiredCapturePoint,
                                      FrameVector2DBasics desiredCapturePointVelocity, FramePoint2DBasics perfectCoP, FramePoint2DBasics perfectCMP)
   {
      boolean inputsAreOk = true;
      if (desiredCapturePoint.containsNaN())
      {
         LogTools.error("Desired ICP contains NaN, setting it to the current ICP and failing.");
         desiredCapturePoint.set(capturePoint);
         inputsAreOk = false;
      }

      if (desiredCapturePointVelocity.containsNaN())
      {
         LogTools.error("Desired ICP Velocity contains NaN, setting it to zero and failing.");
         desiredCapturePointVelocity.setToZero();
         inputsAreOk = false;
      }

      if (perfectCoP.containsNaN())
      {
         LogTools.error("Perfect CoP contains NaN, setting it to the current ICP and failing.");
         perfectCoP.set(capturePoint);
         inputsAreOk = false;
      }

      if (perfectCMP.containsNaN())
      {
         LogTools.error("Perfect CMP contains NaN, setting it to the current ICP and failing.");
         perfectCMP.set(capturePoint);
         inputsAreOk = false;
      }

      return inputsAreOk;
   }

   public void minimizeAngularMomentumRateZ(boolean enable)
   {
      minimizeAngularMomentumRateZ.set(enable);
   }

   public void setFinalDesiredCapturePoint(FramePoint2DReadOnly finalDesiredCapturePoint)
   {
      this.finalDesiredCapturePoint.setIncludingFrame(finalDesiredCapturePoint);
   }

   public void setPerfectCMP(FramePoint2DReadOnly perfectCMP)
   {
      this.perfectCMP.setIncludingFrame(perfectCMP);
   }

   public void setPerfectCoP(FramePoint2DReadOnly perfectCoP)
   {
      this.perfectCoP.setIncludingFrame(perfectCoP);
   }

   /**
    * Sets whether or not to include the momentum rate of change in the vertical direction in the
    * whole body optimization. If false, it will be controlled by attempting to drive the legs to a
    * certain position in the null space
    *
    * @param controlHeightWithMomentum boolean variable on whether or not to control the height with
    *           momentum.
    */
   public void setControlHeightWithMomentum(boolean controlHeightWithMomentum)
   {
      this.controlHeightWithMomentum = controlHeightWithMomentum;
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

   public void clearPlan()
   {
      icpOptimizationController.clearPlan();
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      icpOptimizationController.addFootstepToPlan(footstep, timing);
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      icpOptimizationController.setFinalTransferDuration(finalTransferDuration);
   }

   public void initializeForStanding()
   {
      icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());
   }

   public void initializeForSingleSupport()
   {
      icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0);
   }

   public void initializeForTransfer()
   {
      icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide);
   }

   public void computeCMPInternal(FramePoint2DReadOnly desiredCMPPreviousValue)
   {
      if (perfectCoP.containsNaN())
      {
         perfectCMPDelta.setToZero();
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCMP, capturePoint,
                                           capturePointVelocity, omega0);
      }
      else
      {
         perfectCMPDelta.sub(perfectCMP, perfectCoP);
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMPDelta, capturePoint,
                                           capturePointVelocity, omega0);
      }

      icpOptimizationController.getDesiredCMP(desiredCMP);
      icpOptimizationController.getDesiredCoP(desiredCoP);

      yoUnprojectedDesiredCMP.set(desiredCMP);
   }

   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      if (icpOptimizationController.useStepAdjustment())
      {
         icpOptimizationController.getFootstepSolution(footstepToPack);
      }

      return icpOptimizationController.wasFootstepAdjusted();
   }

   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      icpOptimizationController.submitRemainingTimeInSwingUnderDisturbance(remainingTimeForSwing);
   }

   public ICPOptimizationControllerInterface getICPOptimizationController()
   {
      return icpOptimizationController;
   }

   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {
      icpOptimizationController.submitCurrentPlanarRegions(planarRegions);
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      icpOptimizationController.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }
}
