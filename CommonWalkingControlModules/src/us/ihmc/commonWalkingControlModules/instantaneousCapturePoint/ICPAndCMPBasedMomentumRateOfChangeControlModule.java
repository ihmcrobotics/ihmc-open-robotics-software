package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionMomentControlModule;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class ICPAndCMPBasedMomentumRateOfChangeControlModule
{
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;
   private final GroundReactionMomentControlModule groundReactionMomentControlModule;
   private final CapturabilityBasedDesiredCoPVisualizer visualizer;

   private final DoubleYoVariable kAngularMomentumXY = new DoubleYoVariable("kAngularMomentumXY", registry);
   private final DoubleYoVariable kPelvisAxisAngle = new DoubleYoVariable("kPelvisAxisAngle", registry);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final YoFramePoint2d controlledCoP = new YoFramePoint2d("controlledCoP", "", worldFrame, registry);
   private final YoFramePoint2d controlledCMP = new YoFramePoint2d("controlledCMP", "", worldFrame, registry);
   private final YoFrameVector2d controlledDeltaCMP = new YoFrameVector2d("controlledDeltaCMP", "", worldFrame, registry);

   private final BooleanYoVariable copProjected = new BooleanYoVariable("copProjected", registry);
   private final double totalMass;
   private final double gravityZ;
   private final SpatialForceVector gravitationalWrench;

   private final BooleanYoVariable keepCMPInsideSupportPolygon = new BooleanYoVariable("keepCMPInsideSupportPolygon", registry);

   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);
   private final MomentumCalculator momentumCalculator;

   private final BipedSupportPolygons bipedSupportPolygons;
   private RobotSide supportSide;
   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();

   private double omega0 = 0.0;
   private double desiredCoMHeightAcceleration = 0.0;
   private final FramePoint2d capturePoint = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity = new FrameVector2d();
   private final FramePoint2d finalDesiredCapturePoint = new FramePoint2d();
   
   private final FrameOrientation desiredPelvisOrientation = new FrameOrientation();

   public ICPAndCMPBasedMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         TwistCalculator twistCalculator, double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;
      this.bipedSupportPolygons = bipedSupportPolygons;

      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry, yoGraphicsListRegistry);
      groundReactionMomentControlModule = new GroundReactionMomentControlModule(pelvisFrame, registry);
      groundReactionMomentControlModule.setGains(10.0, 100.0); // kPelvisYaw was 0.0 for M3 movie TODO: move to setGains method

      momentumCalculator = new MomentumCalculator(twistCalculator);
      visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, yoGraphicsListRegistry);
      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());
      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());

      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePoint, finalDesiredCapturePoint, desiredCapturePoint,
            desiredCapturePointVelocity, omega0, keepCMPInsideSupportPolygon.getBooleanValue(), supportPolygon);

      controlledCMP.set(desiredCMP);

      Momentum momentum = new Momentum(centerOfMassFrame);
      momentumCalculator.computeAndPack(momentum);
      FrameVector2d desiredDeltaCMP = determineDesiredDeltaCMP(desiredPelvisOrientation, momentum);
      FramePoint2d desiredCoP = new FramePoint2d(desiredCMP);
      desiredCoP.sub(desiredDeltaCMP);
      desiredCoP.changeFrame(supportPolygon.getReferenceFrame());

      if (supportPolygon.isPointInside(desiredCoP))
      {
         copProjected.set(false);
      }
      else
      {
         supportPolygon.orthogonalProjection(desiredCoP);
         copProjected.set(true);
      }

      desiredCoP.changeFrame(controlledCoP.getReferenceFrame());
      controlledCoP.set(desiredCoP);
      desiredDeltaCMP.sub(desiredCMP, desiredCoP);
      controlledDeltaCMP.set(desiredDeltaCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePoint);
      visualizer.setDesiredCoP(desiredCoP);
      visualizer.setDesiredCMP(desiredCMP);

      supportLegPreviousTick.set(supportSide);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);

      FrameVector normalMoment = groundReactionMomentControlModule.determineGroundReactionMoment(momentum, desiredPelvisOrientation.getYawPitchRoll()[0]);

      SpatialForceVector rateOfChangeOfMomentum = computeTotalGroundReactionWrench(desiredCoP, desiredCMP, fZ, normalMoment);
      rateOfChangeOfMomentum.changeFrame(gravitationalWrench.getExpressedInFrame());
      rateOfChangeOfMomentum.sub(gravitationalWrench);

      momentumRateOfChangeData.set(rateOfChangeOfMomentum);
   }

   private SpatialForceVector computeTotalGroundReactionWrench(FramePoint2d cop2d, FramePoint2d cmp2d, double fZ, FrameVector normalMoment)
   {
      FramePoint centerOfMass = new FramePoint(centerOfMassFrame);
      FramePoint cmp3d = WrenchDistributorTools.computePseudoCMP3d(centerOfMass, cmp2d, fZ, totalMass, omega0);
      FrameVector force = WrenchDistributorTools.computeForce(centerOfMass, cmp3d, fZ);
      force.changeFrame(centerOfMassFrame);

      FramePoint cop3d = cop2d.toFramePoint();
      cop3d.changeFrame(cmp3d.getReferenceFrame());
      cop3d.setZ(cmp3d.getZ());
      FrameVector momentArm = new FrameVector(cop3d);
      momentArm.sub(centerOfMass);

      SpatialForceVector ret = SpatialForceVector.createUsingArm(centerOfMassFrame, force.getVector(), momentArm.getVector());
      normalMoment.changeFrame(ret.getExpressedInFrame());
      ret.addAngularPart(normalMoment.getVector());

      return ret;
   }

   public void setGains(double kAngularMomentumXY, double kPelvisAxisAngle)
   {
      this.kAngularMomentumXY.set(kAngularMomentumXY);
      this.kPelvisAxisAngle.set(kPelvisAxisAngle);
   }

   private FrameVector2d determineDesiredDeltaCMP(FrameOrientation desiredPelvisOrientation, Momentum momentum)
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();

      FrameVector zUnitVector = new FrameVector(frame, 0.0, 0.0, 1.0);

      FrameVector angularMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getAngularPartCopy());
      angularMomentum.changeFrame(frame);

      FrameVector momentumPart = new FrameVector(frame);
      momentumPart.cross(angularMomentum, zUnitVector);
      momentumPart.scale(-kAngularMomentumXY.getDoubleValue());

      Matrix3d desiredPelvisToPelvis = new Matrix3d();
      desiredPelvisOrientation.changeFrame(pelvisFrame);
      desiredPelvisOrientation.getMatrix3d(desiredPelvisToPelvis);

      AxisAngle4d desiredPelvisToPelvisAxisAngle = new AxisAngle4d();
//      desiredPelvisToPelvisAxisAngle.set(desiredPelvisToPelvis);
      RotationFunctions.axisAngleFromMatrix(desiredPelvisToPelvis, desiredPelvisToPelvisAxisAngle);
      FrameVector proportionalPart = new FrameVector(pelvisFrame, desiredPelvisToPelvisAxisAngle.getX(), desiredPelvisToPelvisAxisAngle.getY(), 0.0);
      proportionalPart.scale(desiredPelvisToPelvisAxisAngle.getAngle());
      proportionalPart.changeFrame(frame);
      proportionalPart.cross(proportionalPart, zUnitVector);
      proportionalPart.scale(kPelvisAxisAngle.getDoubleValue());

      FrameVector desiredDeltaCMP = new FrameVector(frame);
      desiredDeltaCMP.add(momentumPart, proportionalPart);
      double maxDeltaDesiredCMP = 0.02;
      if (desiredDeltaCMP.length() > maxDeltaDesiredCMP)
      {
         desiredDeltaCMP.normalize();
         desiredDeltaCMP.scale(maxDeltaDesiredCMP);
      }

      return desiredDeltaCMP.toFrameVector2d();
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
      this.desiredCoMHeightAcceleration = desiredCenterOfMassHeightAcceleration;
   }

   public void setDesiredPelvisOrientation(FrameOrientation desiredPelvisOrientation)
   {
      this.desiredPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
   }

   public void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack)
   {
      momentumRateOfChangeDataToPack.set(momentumRateOfChangeData);
   }

   public void initialize()
   {
      //    empty
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controlledCMP.getFrameTuple2dIncludingFrame(desiredCMPToPack);
   }
}
