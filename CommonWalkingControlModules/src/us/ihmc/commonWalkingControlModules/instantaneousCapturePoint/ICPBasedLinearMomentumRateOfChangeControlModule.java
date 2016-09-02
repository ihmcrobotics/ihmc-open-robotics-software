package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance.Purple;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class ICPBasedLinearMomentumRateOfChangeControlModule
{
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SpatialForceVector desiredMomentumRate = new SpatialForceVector();

   private final DenseMatrix64F linearAndAngularZSelectionMatrix = CommonOps.identity(6);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final YoFrameVector controlledCoMAcceleration;

   private final FrameVector2d achievedCoMAcceleration2d = new FrameVector2d();

   private final double totalMass;
   private final FramePoint centerOfMass;
   private final FramePoint2d centerOfMass2d = new FramePoint2d();
   private final double gravityZ;

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
   private final FramePoint2d perfectCMP = new FramePoint2d();

   private final YoFrameVector defaultLinearMomentumRateWeight = new YoFrameVector("defaultLinearMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector defaultAngularMomentumRateWeight = new YoFrameVector("defaultAngularMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector highLinearMomentumRateWeight = new YoFrameVector("highLinearMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector angularMomentumRateWeight = new YoFrameVector("angularMomentumRateWeigth", worldFrame, registry);
   private final YoFrameVector linearMomentumRateWeight = new YoFrameVector("linearMomentumRateWeight", worldFrame, registry);

   private final BooleanYoVariable minimizeAngularMomentumRateZ = new BooleanYoVariable("minimizeAngularMomentumRateZ", registry);

   private final CMPProjector cmpProjector;
   private final FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();

   private final YoFramePoint2d yoUnprojectedDesiredCMP = new YoFramePoint2d("unprojectedDesiredCMP", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoSafeAreaPolygon = new YoFrameConvexPolygon2d("yoSafeAreaPolygon", worldFrame, 10, registry);
   private final YoFrameConvexPolygon2d yoProjectionPolygon = new YoFrameConvexPolygon2d("yoProjectionPolygon", worldFrame, 10, registry);

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, parentRegistry, yoGraphicsListRegistry, true);
   }

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      if (use2DProjection)
         cmpProjector = new SmartCMPProjectorTwo(registry);
      else
         cmpProjector = new SmartCMPPlanarProjector(registry);

      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry);
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.bipedSupportPolygons = bipedSupportPolygons;

      this.totalMass = totalMass;
      centerOfMass = new FramePoint(centerOfMassFrame);
      this.gravityZ = gravityZ;
      parentRegistry.addChild(registry);

      controlledCoMAcceleration = new YoFrameVector("controlledCoMAcceleration", "", centerOfMassFrame, registry);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
      linearMomentumRateWeight.set(defaultLinearMomentumRateWeight);
      momentumRateCommand.setWeights(0.0, 0.0, 0.0, linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());

      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);
      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);

      if (yoGraphicsListRegistry != null)
      {
         String graphicListName = getClass().getSimpleName();
         YoGraphicPosition unprojectedDesiredCMPViz = new YoGraphicPosition("Unprojected Desired CMP", yoUnprojectedDesiredCMP, 0.008, Purple(), GraphicType.BALL_WITH_ROTATED_CROSS);
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

   public void setMomentumWeight(Vector3d angularWeight, Vector3d linearWeight)
   {
      defaultLinearMomentumRateWeight.set(linearWeight);
      defaultAngularMomentumRateWeight.set(angularWeight);
   }

   public void setMomentumWeight(Vector3d linearWeight)
   {
      defaultLinearMomentumRateWeight.set(linearWeight);
   }

   public void setAngularMomentumWeight(Vector3d angularWeight)
   {
      defaultAngularMomentumRateWeight.set(angularWeight);
   }

   public void setHighMomentumWeightForRecovery(Vector3d highLinearWeight)
   {
      highLinearMomentumRateWeight.set(highLinearWeight);
   }

   private final BooleanYoVariable desiredCMPinSafeArea = new BooleanYoVariable("DesiredCMPinSafeArea", registry);

   public void compute(FramePoint2d desiredCMPPreviousValue, FramePoint2d desiredCMPToPack)
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(desiredCMPPreviousValue, capturePoint, desiredCapturePoint,
            finalDesiredCapturePoint, desiredCapturePointVelocity, perfectCMP, omega0);

      yoUnprojectedDesiredCMP.set(desiredCMP);

      // do projection here:
      if (!areaToProjectInto.isEmpty())
      {
         desiredCMPinSafeArea.set(safeArea.isPointInside(desiredCMP));
         if (safeArea.isPointInside(desiredCMP))
         {
            supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
            areaToProjectInto.setIncludingFrameAndUpdate(supportPolygon);
         }

         cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
         if (cmpProjector.getWasCMPProjected())
            icpProportionalController.bleedOffIntegralTerm();
      }

      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);
      if (desiredCMP.containsNaN())
      {
         desiredCMP.set(capturePoint);
         System.err.println(getClass().getSimpleName() + ": desiredCMP contained NaN. Set it to capturePoint.");
      }

      desiredCMPToPack.setIncludingFrame(desiredCMP);
      desiredCMPToPack.changeFrame(worldFrame);

      supportLegPreviousTick.set(supportSide);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);

      if (minimizeAngularMomentumRateZ.getBooleanValue())
      {
         desiredMomentumRate.setToZero(centerOfMassFrame);
         desiredMomentumRate.setLinearPart(linearMomentumRateOfChange);
         momentumRateCommand.set(desiredMomentumRate);
         momentumRateCommand.setSelectionMatrix(linearAndAngularZSelectionMatrix);
      }
      else
      {
         momentumRateCommand.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
      }

      momentumRateCommand.setWeights(angularMomentumRateWeight.getX(), angularMomentumRateWeight.getY(), angularMomentumRateWeight.getZ(),
            linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());
   }

   public void computeAchievedCMP(FrameVector achievedLinearMomentumRate, FramePoint2d achievedCMPToPack)
   {
      if (achievedLinearMomentumRate.containsNaN())
         return;

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setByProjectionOntoXYPlaneIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / totalMass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMPToPack.set(achievedCoMAcceleration2d);
      achievedCMPToPack.scale(-1.0 / (omega0 * omega0));
      achievedCMPToPack.add(centerOfMass2d);
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();

   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
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

   public void setPerfectCMP(FramePoint2d perfectCMP)
   {
      this.perfectCMP.setIncludingFrame(perfectCMP);
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

   public void setCMPProjectionArea(FrameConvexPolygon2d areaToProjectInto, FrameConvexPolygon2d safeArea)
   {
      this.areaToProjectInto.setIncludingFrameAndUpdate(areaToProjectInto);
      this.safeArea.setIncludingFrameAndUpdate(safeArea);

      yoSafeAreaPolygon.setFrameConvexPolygon2d(safeArea);
      yoProjectionPolygon.setFrameConvexPolygon2d(areaToProjectInto);
   }

   public void minimizeAngularMomentumRateZ(boolean enable)
   {
      minimizeAngularMomentumRateZ.set(enable);
   }
}
