package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ICPPlannerWithAngularMomentumOffset extends ICPPlannerWithTimeFreezer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final String namePrefix = "icpPlanner";

   private final YoFrameVector modifiedICPVelocity;
   private final YoFrameVector modifiedICPAcceleration;

   private final YoFramePoint modifiedCMPPosition;
   private final YoFrameVector modifiedCMPVelocity;

   private final DoubleYoVariable modifiedTimeInCurrentState;
   private final DoubleYoVariable modifiedTimeInCurrentStateRemaining;

   private final double mass;
   private final double gravityZ;

   public ICPPlannerWithAngularMomentumOffset(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                              CapturePointPlannerParameters capturePointPlannerParameters, double mass, double gravityZ,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, parentRegistry, yoGraphicsListRegistry);

      this.mass = mass;
      this.gravityZ = gravityZ;

      modifiedICPVelocity = new YoFrameVector(namePrefix + "ModifiedCapturePointVelocity", worldFrame, registry);
      modifiedICPAcceleration = new YoFrameVector(namePrefix + "ModifiedCapturePointAcceleration", worldFrame, registry);

      modifiedCMPPosition = new YoFramePoint(namePrefix + "ModifiedCMPPosition", worldFrame, registry);
      modifiedCMPVelocity = new YoFrameVector(namePrefix + "ModifiedCMPVelocity", worldFrame, registry);

      modifiedTimeInCurrentState = new DoubleYoVariable(namePrefix + "ModifiedTimeInCurrentState", registry);
      modifiedTimeInCurrentStateRemaining = new DoubleYoVariable(namePrefix + "ModifiedRemainingTime", registry);
   }

   //todo make a gain that we skew this with
   public void modifyDesiredICPForCMPOffset(FramePoint desiredCoP)
   {
      desiredCoP.changeFrame(worldFrame);
      modifiedCMPPosition.set(desiredCoP);

      estimateCurrentTimeWithModifiedCMP(desiredCoP);

      double omega0 = this.omega0.getDoubleValue();
      modifiedICPVelocity.set(desiredICPPosition);
      modifiedICPVelocity.sub(modifiedCMPPosition);
      modifiedICPVelocity.scale(omega0);

      CapturePointTools.computeDesiredCapturePointAcceleration(omega0, modifiedICPVelocity, modifiedICPAcceleration);
      CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(modifiedICPVelocity, modifiedICPAcceleration, omega0, modifiedCMPVelocity);
   }

   //todo make a gain that we skew this with
   public void modifyDesiredICPForAngularMomentum(FrameVector angularMomentumRate)
   {
      CapturePointTools.computeCentroidalMomentumPivot(mass, gravityZ, angularMomentumRate, desiredCMPPosition, modifiedCMPPosition); //todo incorporate vertical acceleration

      estimateCurrentTimeWithModifiedCMP(modifiedCMPPosition.getFrameTuple());

      double omega0 = this.omega0.getDoubleValue();
      modifiedICPVelocity.set(desiredICPPosition);
      modifiedICPVelocity.sub(modifiedCMPPosition);
      modifiedICPVelocity.scale(omega0);

      CapturePointTools.computeDesiredCapturePointAcceleration(omega0, modifiedICPVelocity, modifiedICPAcceleration);
      CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(modifiedICPVelocity, modifiedICPAcceleration, omega0, modifiedCMPVelocity);
   }

   private void estimateCurrentTimeWithModifiedCMP(FramePoint desiredCoPFromAngularMomentum)
   {
      double copCMPDistance = desiredCMPPosition.getXYPlaneDistance(desiredCoPFromAngularMomentum);
      double distanceFromCMP = desiredICPPosition.getXYPlaneDistance(modifiedCMPPosition);

      double modifiedTimeInState = 1.0 / omega0.getDoubleValue() * Math.log(distanceFromCMP / copCMPDistance);
      modifiedTimeInCurrentState.set(modifiedTimeInState);
      modifiedTimeInCurrentStateRemaining.set(getCurrentStateDuration() - modifiedTimeInCurrentState.getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector desiredCapturePointVelocityToPack)
   {
      modifiedICPVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocityToPack)
   {
      modifiedICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(modifiedICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      modifiedCMPPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      modifiedCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      modifiedCMPVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack)
   {
      modifiedCMPVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentState()
   {
      return modifiedTimeInCurrentState.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentStateRemaining()
   {
      return modifiedTimeInCurrentStateRemaining.getDoubleValue();
   }
}
