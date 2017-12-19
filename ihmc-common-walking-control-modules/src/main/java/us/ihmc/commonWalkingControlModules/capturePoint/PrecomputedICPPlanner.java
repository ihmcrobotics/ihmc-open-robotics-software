package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.*;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PrecomputedICPPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoFramePoint yoDesiredCMPPosition = new YoFramePoint(name + "DesiredCMPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoDesiredCoMPosition = new YoFramePoint(name + "DesiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoDesiredICPPosition = new YoFramePoint(name + "DesiredICPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector yoDesiredICPVelocity = new YoFrameVector(name + "DesiredICPVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoBoolean currentlyBlendingICPTrajectories = new YoBoolean("currentlyBlendingICPTrajectories", registry);
   private final YoBoolean isBlending = new YoBoolean("isBlending", registry);
   private final YoDouble blendingStartTime = new YoDouble("blendingStartTime", registry);
   private final YoDouble blendingDuration = new YoDouble("blendingDuration", registry);
   private final FramePoint2D precomputedDesiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D precomputedDesiredCapturePointVelocity2d = new FrameVector2D();

   private final YoDouble omega0 = new YoDouble(name + "Omega0", registry);

   private final FramePoint3D desiredICPPosition = new FramePoint3D();
   private final FrameVector3D desiredICPVelocity = new FrameVector3D();
   private final FramePoint3D desiredCoMPosition = new FramePoint3D();

   private final CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler;

   public PrecomputedICPPlanner(CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler, YoVariableRegistry parentRegistry,
                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.centerOfMassTrajectoryHandler = centerOfMassTrajectoryHandler;
      blendingDuration.set(0.5);

      parentRegistry.addChild(registry);

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

      YoGraphicPosition desiredCMPPositionViz = new YoGraphicPosition("Perfect CMP Precomputed", yoDesiredCMPPosition, 0.005, BlueViolet());

      yoGraphicsList.add(desiredCMPPositionViz);
      artifactList.add(desiredCMPPositionViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);

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
   }

   public void compute(double time, FramePoint2D desiredCapturePoint2dToPack, FrameVector2D desiredCapturePointVelocity2dToPack, FramePoint2D desiredCMPToPack)
   {
      if (isWithinInterval(time))
      {
         compute(time);
         yoDesiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePoint2dToPack);
         yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocity2dToPack);
         yoDesiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCMPToPack);
      }
      else
      {
         hideViz();
      }
      currentlyBlendingICPTrajectories.set(false);
   }

   public void computeAndBlend(double time, FramePoint2D desiredCapturePoint2d, FrameVector2D desiredCapturePointVelocity2d, FramePoint2D desiredCMP)
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

         desiredCapturePoint2d.interpolate(desiredCapturePoint2d, precomputedDesiredCapturePoint2d, alpha);
         desiredCapturePointVelocity2d.interpolate(desiredCapturePointVelocity2d, precomputedDesiredCapturePointVelocity2d, alpha);
         computeDesiredCentroidalMomentumPivot(desiredCapturePoint2d, desiredCapturePointVelocity2d, omega0.getDoubleValue(), desiredCMP);
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
      yoDesiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
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
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
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
      yoDesiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   private void hideViz()
   {
      yoDesiredICPPosition.setToNaN();
      yoDesiredICPVelocity.setToNaN();
      yoDesiredCoMPosition.setToNaN();
      yoDesiredCMPPosition.setToNaN();
   }
}
