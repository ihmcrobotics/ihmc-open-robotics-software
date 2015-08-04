package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class ICPBasedLinearMomentumRateOfChangeControlModule
{
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;
   private final CapturabilityBasedDesiredCoPVisualizer visualizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final YoFramePoint2d controlledCMP = new YoFramePoint2d("controlledCMP", "", worldFrame, registry);
   private final YoFrameVector controlledCoMAcceleration;

   private final double totalMass;
   private final FramePoint centerOfMass;
   private final double gravityZ;

   private final BooleanYoVariable keepCMPInsideSupportPolygon = new BooleanYoVariable("keepCMPInsideSupportPolygon", registry);

   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);

   private final BipedSupportPolygons bipedSupportPolygons;
   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
   private RobotSide supportSide = null;

   private double desiredCoMHeightAcceleration = 0.0;
   private double omega0 = 0.0;
   private final FramePoint2d capturePoint = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity = new FrameVector2d();
   private final FramePoint2d finalDesiredCapturePoint = new FramePoint2d();

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry, yoGraphicsListRegistry);
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.bipedSupportPolygons = bipedSupportPolygons;

      visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, yoGraphicsListRegistry);
      this.totalMass = totalMass;
      centerOfMass = new FramePoint(centerOfMassFrame);
      this.gravityZ = gravityZ;
      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      parentRegistry.addChild(registry);

      // hide CoP since we won't be calculating it explicitly in this class
      visualizer.setDesiredCoP(new FramePoint2d(controlledCMP.getReferenceFrame(), Double.NaN, Double.NaN));

      controlledCoMAcceleration = new YoFrameVector("controlledCoMAcceleration", "", centerOfMassFrame, registry);

   }

   public void initialize()
   {
   }

   public void compute()
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());

      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePoint, desiredCapturePoint, finalDesiredCapturePoint,
            desiredCapturePointVelocity, omega0, keepCMPInsideSupportPolygon.getBooleanValue(), supportPolygon);

      desiredCMP.changeFrame(controlledCMP.getReferenceFrame());
      controlledCMP.set(desiredCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePoint);
      visualizer.setDesiredCMP(desiredCMP);
      visualizer.setFinalDesiredCapturePoint(finalDesiredCapturePoint);

      supportLegPreviousTick.set(supportSide);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);
      momentumRateOfChangeData.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
   }

   public void updateCenterOfMassViz()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      visualizer.setCenterOfMass(centerOfMass);
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();

   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      visualizer.setPseudoCMP(cmp3d);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controlledCMP.getFrameTuple2dIncludingFrame(desiredCMPToPack);
   }

   public void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack)
   {
      momentumRateOfChangeDataToPack.set(momentumRateOfChangeData);
   }

   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
   }

   public void setCapturePoint(FramePoint2d capturePoint)
   {
      this.capturePoint.setIncludingFrame(capturePoint);
   }

   public void setOmega0(double omega0)
   {
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      this.omega0 = omega0;
   }

   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public void setDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public void setFinalDesiredCapturePoint(FramePoint2d finalDesiredCapturePoint)
   {
      this.finalDesiredCapturePoint.setIncludingFrame(finalDesiredCapturePoint);
   }

   public void keepCMPInsideSupportPolygon(boolean keepCMPInsideSupportPolygon)
   {
      this.keepCMPInsideSupportPolygon.set(keepCMPInsideSupportPolygon);
   }

   public void setDesiredCenterOfMassHeightAcceleration(double desiredCenterOfMassHeightAcceleration)
   {
      desiredCoMHeightAcceleration = desiredCenterOfMassHeightAcceleration;
   }
}
