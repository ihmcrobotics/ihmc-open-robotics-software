package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

import javax.vecmath.Vector3d;

import static us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance.Purple;

public class ICPOptimizationLinearMomentumRateOfChangeControlModule
{
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SpatialForceVector desiredMomentumRate = new SpatialForceVector();

   private final DenseMatrix64F linearAndAngularZSelectionMatrix = CommonOps.identity(6);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPOptimizationController icpOptimizationController;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final YoFrameVector controlledCoMAcceleration;

   private final FrameVector2d achievedCoMAcceleration2d = new FrameVector2d();

   private final DoubleYoVariable yoTime;

   private final double totalMass;
   private final FramePoint centerOfMass;
   private final FramePoint2d centerOfMass2d = new FramePoint2d();
   private final double gravityZ;
   private double omega0;

   private RobotSide supportLegSide = RobotSide.LEFT;
   private RobotSide transferToSide = RobotSide.LEFT;

   private double desiredCoMHeightAcceleration = 0.0;
   private final FramePoint2d capturePoint = new FramePoint2d();
   private final FramePoint2d desiredCapturePoint = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity = new FrameVector2d();

   private final YoFrameVector defaultLinearMomentumRateWeight = new YoFrameVector("defaultLinearMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector defaultAngularMomentumRateWeight = new YoFrameVector("defaultAngularMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector highLinearMomentumRateWeight = new YoFrameVector("highLinearMomentumRateWeight", worldFrame, registry);
   private final YoFrameVector angularMomentumRateWeight = new YoFrameVector("angularMomentumRateWeigth", worldFrame, registry);
   private final YoFrameVector linearMomentumRateWeight = new YoFrameVector("linearMomentumRateWeight", worldFrame, registry);

   private final BooleanYoVariable minimizeAngularMomentumRateZ = new BooleanYoVariable("minimizeAngularMomentumRateZ", registry);

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters,
         ICPOptimizationParameters icpOptimizationParameters, DoubleYoVariable yoTime, double totalMass, double gravityZ, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;
      this.yoTime = yoTime;

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      icpOptimizationController = new ICPOptimizationController(icpPlannerParameters, icpOptimizationParameters, bipedSupportPolygons, contactableFeet,
            registry, yoGraphicsListRegistry);

      centerOfMass = new FramePoint(centerOfMassFrame);
      parentRegistry.addChild(registry);

      controlledCoMAcceleration = new YoFrameVector("controlledCoMAcceleration", "", centerOfMassFrame, registry);
      angularMomentumRateWeight.set(defaultAngularMomentumRateWeight);
      linearMomentumRateWeight.set(defaultLinearMomentumRateWeight);
      momentumRateCommand.setWeights(0.0, 0.0, 0.0, linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());

      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);
      MatrixTools.removeRow(linearAndAngularZSelectionMatrix, 0);
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

   public void setDoubleSupportDuration(double doubleSupportDuration)
   {
      icpOptimizationController.setDoubleSupportDuration(doubleSupportDuration);
   }

   public void setSingleSupportDuration(double singleSupportDuration)
   {
      icpOptimizationController.setSingleSupportDuration(singleSupportDuration);
   }

   public void setSupportLegSide(RobotSide supportLegSide)
   {
      this.supportLegSide = supportLegSide;
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

   public void addFootstepToPlan(Footstep footstep)
   {
      icpOptimizationController.addFootstepToPlan(footstep);
   }

   public void initializeForStanding()
   {
      icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());
   }

   public void initializeForSingleSupport()
   {
      icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportLegSide, omega0);
   }

   public void initializeForTransfer()
   {
      icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide, omega0);
   }

   private final FramePoint2d desiredCMP = new FramePoint2d();
   public void compute(FramePoint2d desiredCMPToPack)
   {
      icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, capturePoint, omega0);
      icpOptimizationController.getDesiredCMP(desiredCMP);

      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);
      if (desiredCMP.containsNaN())
      {
         desiredCMP.set(capturePoint);
         System.err.println(getClass().getSimpleName() + ": desiredCMP contained NaN. Set it to capturePoint.");
      }

      desiredCMPToPack.setIncludingFrame(desiredCMP);
      desiredCMPToPack.changeFrame(worldFrame);

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

   public void setOmega0(double omega0)
   {
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      this.omega0 = omega0;
   }

   public void setCapturePoint(FramePoint2d capturePoint)
   {
      this.capturePoint.setIncludingFrame(capturePoint);
   }

   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public void setDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocity)
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

   private final FramePose footstepPose = new FramePose();
   private final FramePoint2d footstepPositionSolution = new FramePoint2d();

   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      if (icpOptimizationController.getNumberOfFootstepsToConsider() > 0)
      {
         footstepToPack.getPose(footstepPose);
         icpOptimizationController.getFootstepSolution(0, footstepPositionSolution);
         footstepPose.setXYFromPosition2d(footstepPositionSolution);
         footstepPose.setPose(footstepPose);
      }

      return icpOptimizationController.wasFootstepAdjusted();
   }
}
