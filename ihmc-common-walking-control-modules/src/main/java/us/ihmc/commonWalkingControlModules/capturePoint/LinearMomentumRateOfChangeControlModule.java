package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public abstract class LinearMomentumRateOfChangeControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry;

   private Vector3DReadOnly defaultLinearMomentumRateWeight;
   private Vector3DReadOnly defaultAngularMomentumRateWeight;
   private Vector3DReadOnly highLinearMomentumRateWeight;
   private final YoFrameVector3D angularMomentumRateWeight;
   private final YoFrameVector3D linearMomentumRateWeight;

   private final YoBoolean minimizeAngularMomentumRateZ;

   private final YoFrameVector3D controlledCoMAcceleration;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D linearAndAngularZSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D linearXYSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D linearXYAndAngularZSelectionMatrix = new SelectionMatrix6D();

   protected double omega0 = 0.0;
   private double totalMass;
   private double gravityZ;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMass;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   protected final FramePoint2D capturePoint = new FramePoint2D();
   protected final FrameVector2D capturePointVelocity = new FrameVector2D();
   protected final FramePoint2D desiredCapturePoint = new FramePoint2D();
   protected final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();
   protected final FramePoint2D finalDesiredCapturePoint = new FramePoint2D();

   protected final FramePoint2D perfectCMP = new FramePoint2D();
   protected final FramePoint2D perfectCoP = new FramePoint2D();
   protected final FramePoint2D desiredCMP = new FramePoint2D();

   protected final FrameConvexPolygon2D areaToProjectInto = new FrameConvexPolygon2D();
   protected final FrameConvexPolygon2D safeArea = new FrameConvexPolygon2D();

   private boolean controlHeightWithMomentum;

   private final YoFrameConvexPolygon2D yoSafeAreaPolygon;
   private final YoFrameConvexPolygon2D yoProjectionPolygon;

   protected final YoFramePoint2D yoUnprojectedDesiredCMP;
   protected final CMPProjector cmpProjector;

   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();
   private double desiredCoMHeightAcceleration = 0.0;

   public LinearMomentumRateOfChangeControlModule(String namePrefix, ReferenceFrames referenceFrames, double gravityZ, double totalMass,
                                                  YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2dProjection)
   {
      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.totalMass = totalMass;
      this.gravityZ = gravityZ;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());


      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMass = new FramePoint3D(centerOfMassFrame);

      controlledCoMAcceleration = new YoFrameVector3D(namePrefix + "ControlledCoMAcceleration", "", centerOfMassFrame, registry);

      angularMomentumRateWeight = new YoFrameVector3D(namePrefix + "AngularMomentumRateWeight", worldFrame, registry);
      linearMomentumRateWeight = new YoFrameVector3D(namePrefix + "LinearMomentumRateWeight", worldFrame, registry);

      minimizeAngularMomentumRateZ = new YoBoolean(namePrefix + "MinimizeAngularMomentumRateZ", registry);

      yoSafeAreaPolygon = new YoFrameConvexPolygon2D("yoSafeAreaPolygon", worldFrame, 10, registry);
      yoProjectionPolygon = new YoFrameConvexPolygon2D("yoProjectionPolygon", worldFrame, 10, registry);

      linearAndAngularZSelectionMatrix.selectAngularX(false);
      linearAndAngularZSelectionMatrix.selectAngularY(false);

      linearXYSelectionMatrix.setToLinearSelectionOnly();
      linearXYSelectionMatrix.selectLinearZ(false); // remove height

      linearXYAndAngularZSelectionMatrix.setToLinearSelectionOnly();
      linearXYAndAngularZSelectionMatrix.selectLinearZ(false); // remove height
      linearXYAndAngularZSelectionMatrix.selectAngularZ(true);

      momentumRateCommand.setWeights(0.0, 0.0, 0.0, linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());

      perfectCoP.setToNaN();

      yoUnprojectedDesiredCMP = new YoFramePoint2D("unprojectedDesiredCMP", ReferenceFrame.getWorldFrame(), registry);

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
    * Sets the capture point position to be used for the next call of
    * {@link #compute(FramePoint2DReadOnly, FramePoint2D)}.
    * <p>
    * Note that the internal value for the capture point velocity is set to {@link Double#NaN} when
    * calling this method. If an actual value is to be passed for the velocity, call
    * {@link #setCapturePoint(FramePoint2DReadOnly, FrameVector2DReadOnly)} instead.
    * </p>
    * 
    * @param capturePoint the measured position of the capture point. Not modified.
    */
   public void setCapturePoint(FramePoint2DReadOnly capturePoint)
   {
      setCapturePoint(capturePoint, null);
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
      if (capturePointVelocity != null)
         this.capturePointVelocity.setIncludingFrame(capturePointVelocity);
      else
         this.capturePointVelocity.setToNaN();
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

   private final FramePoint3D cmp3d = new FramePoint3D();
   private final FrameVector3D groundReactionForce = new FrameVector3D();

   protected FrameVector3D computeGroundReactionForce(FramePoint2DReadOnly cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   private boolean desiredCMPcontainedNaN = false;

   public void compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack)
   {
      computeCMPInternal(desiredCMPPreviousValue);

      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);
      if (desiredCMP.containsNaN())
      {
         if (!desiredCMPcontainedNaN)
            PrintTools.error("Desired CMP containes NaN, setting it to the ICP - only showing this error once");
         desiredCMP.set(capturePoint);
         desiredCMPcontainedNaN = true;
      }
      else
      {
         desiredCMPcontainedNaN = false;
      }

      desiredCMPToPack.setIncludingFrame(desiredCMP);
      desiredCMPToPack.changeFrame(worldFrame);

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

      if (minimizeAngularMomentumRateZ.getBooleanValue())
      {
         if (!controlHeightWithMomentum)
            momentumRateCommand.setSelectionMatrix(linearXYAndAngularZSelectionMatrix);
         else
            momentumRateCommand.setSelectionMatrix(linearAndAngularZSelectionMatrix);
      }
      else
      {
         if (!controlHeightWithMomentum)
            momentumRateCommand.setSelectionMatrix(linearXYSelectionMatrix);
         else
            momentumRateCommand.setSelectionMatrixForLinearControl();
      }

      momentumRateCommand.setWeights(angularMomentumRateWeight.getX(), angularMomentumRateWeight.getY(), angularMomentumRateWeight.getZ(),
                                     linearMomentumRateWeight.getX(), linearMomentumRateWeight.getY(), linearMomentumRateWeight.getZ());
   }

   public void setCMPProjectionArea(FrameConvexPolygon2DReadOnly areaToProjectInto, FrameConvexPolygon2DReadOnly safeArea)
   {
      this.areaToProjectInto.setIncludingFrame(areaToProjectInto);
      this.safeArea.setIncludingFrame(safeArea);

      yoSafeAreaPolygon.set(safeArea);
      yoProjectionPolygon.set(areaToProjectInto);
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

   public abstract void computeCMPInternal(FramePoint2DReadOnly desiredCMPPreviousValue);

   public abstract void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon);
}
