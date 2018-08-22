package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCentroidalMomentumPivot;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class PrecomputedICPPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoFramePoint3D yoDesiredCoPPosition = new YoFramePoint3D(name + "DesiredCoPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredCMPPosition = new YoFramePoint3D(name + "DesiredCMPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D(name + "DesiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredICPPosition = new YoFramePoint3D(name + "DesiredICPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoDesiredICPVelocity = new YoFrameVector3D(name + "DesiredICPVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoBoolean currentlyBlendingICPTrajectories = new YoBoolean("currentlyBlendingICPTrajectories", registry);
   private final YoBoolean isBlending = new YoBoolean("isBlending", registry);
   private final YoDouble blendingStartTime = new YoDouble("blendingStartTime", registry);
   private final YoDouble blendingDuration = new YoDouble("blendingDuration", registry);
   private final FramePoint2D precomputedDesiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D precomputedDesiredCapturePointVelocity2d = new FrameVector2D();
   private final FrameVector2D precomputedCenterOfPressure2d = new FrameVector2D();

   private final YoDouble omega0 = new YoDouble(name + "Omega0", registry);

   private final FramePoint3D desiredICPPosition = new FramePoint3D();
   private final FrameVector3D desiredICPVelocity = new FrameVector3D();
   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRate = new FrameVector3D();

   private final CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler;
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;

   private double comZAcceleration;
   private double mass;
   private double gravity;

   public PrecomputedICPPlanner(CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler, MomentumTrajectoryHandler momentumTrajectoryHandler,
                                YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.centerOfMassTrajectoryHandler = centerOfMassTrajectoryHandler;
      this.momentumTrajectoryHandler = momentumTrajectoryHandler;
      blendingDuration.set(0.5);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
         ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
         
         YoGraphicPosition desiredICPPositionGraphic = new YoGraphicPosition("Desired ICP Precomputed", yoDesiredICPPosition, 0.005, Yellow(),
                                                                             GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsList.add(desiredICPPositionGraphic);
         artifactList.add(desiredICPPositionGraphic.createArtifact());
         
         YoGraphicPosition desiredCenterOfMassPositionViz = new YoGraphicPosition("Desired CoM Precomputed", yoDesiredCoMPosition, 0.003, Black(),
                                                                                  GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsList.add(desiredCenterOfMassPositionViz);
         artifactList.add(desiredCenterOfMassPositionViz.createArtifact());
         
         YoGraphicPosition desiredCoPPositionViz = new YoGraphicPosition("Perfect CoP Precomputed", yoDesiredCoPPosition, 0.005, BlueViolet(),
                                                                         GraphicType.DIAMOND);
         YoGraphicPosition desiredCMPPositionViz = new YoGraphicPosition("Perfect CMP Precomputed", yoDesiredCMPPosition, 0.005, BlueViolet());
         
         artifactList.add(desiredCoPPositionViz.createArtifact());
         yoGraphicsList.add(desiredCMPPositionViz);
         artifactList.add(desiredCMPPositionViz.createArtifact());
         
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      hideViz();
   }

   private void compute(double time)
   {
      double omega0 = this.omega0.getDoubleValue();
      centerOfMassTrajectoryHandler.packDesiredICPAtTime(time, omega0, desiredICPPosition, desiredICPVelocity, desiredCoMPosition);

      computeDesiredCentroidalMomentumPivot(desiredICPPosition, desiredICPVelocity, omega0, yoDesiredCMPPosition);

      precomputedDesiredCapturePoint2d.set(desiredICPPosition);
      precomputedDesiredCapturePointVelocity2d.set(desiredICPVelocity);

      yoDesiredICPPosition.set(desiredICPPosition);
      yoDesiredICPVelocity.set(desiredICPVelocity);
      yoDesiredCoMPosition.set(desiredCoMPosition);

      yoDesiredCoPPosition.set(yoDesiredCMPPosition);
      // Can compute CoP if we have a momentum rate of change otherwise set it to match the CMP.
      if (momentumTrajectoryHandler != null && momentumTrajectoryHandler.packDesiredAngularMomentumAtTime(time, desiredAngularMomentum, desiredAngularMomentumRate))
      {
         double fZ = WrenchDistributorTools.computeFz(mass, gravity, comZAcceleration);
         yoDesiredCoPPosition.addX(-desiredAngularMomentumRate.getY() / fZ);
         yoDesiredCoPPosition.addY(desiredAngularMomentumRate.getX() / fZ);
      }
      precomputedCenterOfPressure2d.set(yoDesiredCMPPosition);
   }

   public void compute(double time, FramePoint2D desiredCapturePoint2dToPack, FrameVector2D desiredCapturePointVelocity2dToPack,
                       FixedFramePoint2DBasics desiredCoP2DToPack)
   {
      if (isWithinInterval(time))
      {
         compute(time);
         desiredCapturePoint2dToPack.setIncludingFrame(yoDesiredICPPosition);
         desiredCapturePointVelocity2dToPack.setIncludingFrame(yoDesiredICPVelocity);
         // TODO desired CoP
      }
      else
      {
         hideViz();
      }
      currentlyBlendingICPTrajectories.set(false);
   }

   public void computeAndBlend(double time, FixedFramePoint2DBasics desiredCapturePoint2dToPack, FixedFrameVector2DBasics desiredCapturePointVelocity2dToPack,
                               FixedFramePoint2DBasics desiredCenterOfPressure2dToPack)
   {
      if (isWithinInterval(time))
      {
         compute(time);
         if (!currentlyBlendingICPTrajectories.getBooleanValue())
         {
            blendingStartTime.set(time);
            currentlyBlendingICPTrajectories.set(true);
         }

         double alpha = (time - blendingStartTime.getDoubleValue()) / blendingDuration.getDoubleValue();
         isBlending.set(alpha < 1.0);
         alpha = MathTools.clamp(alpha, 0.0, 1.0);

         desiredCapturePoint2dToPack.interpolate(desiredCapturePoint2dToPack, precomputedDesiredCapturePoint2d, alpha);
         desiredCapturePointVelocity2dToPack.interpolate(desiredCapturePointVelocity2dToPack, precomputedDesiredCapturePointVelocity2d, alpha);
         desiredCenterOfPressure2dToPack.interpolate(desiredCenterOfPressure2dToPack, precomputedCenterOfPressure2d, alpha);
      }
      else
      {
         isBlending.set(false);
         currentlyBlendingICPTrajectories.set(false);
         hideViz();
      }
   }

   public boolean isWithinInterval(double time)
   {
      return centerOfMassTrajectoryHandler.isWithinInterval(time);
   }

   /**
    * Intrinsic robot parameter.
    * <p>
    * Correspond the natural frequency response of the robot when modeled as an inverted pendulum:
    * {@code omega0 = Math.sqrt(g / z0)}, where {@code g} is equal to the magnitude of the gravity,
    * and {@code z0} is the constant center of mass height of the robot with respect to is feet.
    * </p>
    *
    * @param omega0 the robot's natural frequency.
    */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   public void setCoMZAcceleration(double comZAcceleration)
   {
      this.comZAcceleration = comZAcceleration;
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setGravity(double gravity)
   {
      this.gravity = gravity;
   }

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   public void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.setIncludingFrame(yoDesiredICPPosition);
   }

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(yoDesiredICPVelocity);
   }

   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(yoDesiredCMPPosition);
   }

   private void hideViz()
   {
      yoDesiredICPPosition.setToNaN();
      yoDesiredICPVelocity.setToNaN();
      yoDesiredCoMPosition.setToNaN();
      yoDesiredCMPPosition.setToNaN();
   }
}
