package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.geometry.AngleTools.computeAngleDifferenceMinusPiToPi;
import static us.ihmc.robotics.geometry.AngleTools.trimAngleMinusPiToPi;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
* @author twan
*         Date: 6/12/13
*/
public class CirclePoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final YoDouble currentTime;
   private final YoPolynomial anglePolynomial;

   private final DoubleProvider desiredTrajectoryTimeProvider;

   private final YoDouble desiredTrajectoryTime;
   private final YoDouble initialRadius;
   private final YoDouble initialZ;
   private final YoDouble initialAngle;
   private final YoDouble currentRelativeAngle;

   private final YoBoolean isCurrentAngleBeingAdjusted;
   private final YoDouble maximumAngleTrackingErrorTolerated;
   private final YoDouble currentControlledFrameRelativeAngle;
   private final YoDouble currentAngleTrackingError;
   private final YoDouble currentAdjustedRelativeAngle;

   private final YoDouble desiredRotationAngle;
   private final YoDouble finalAngle;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FramePoint3D currentPosition = new FramePoint3D();
   private final FramePoint3D finalPosition = new FramePoint3D();

   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameQuaternion currentOrientation = new FrameQuaternion();

   private final FrameVector3D currentVelocity = new FrameVector3D();
   private final FrameVector3D currentAcceleration = new FrameVector3D();

   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D currentAngularAcceleration = new FrameVector3D();

   private final YoFramePoint3D yoInitialPosition;
   private final YoFramePoint3D yoFinalPosition;

   private final YoFramePoint3D yoCurrentPosition;
   private final YoFrameVector3D yoCurrentVelocity;
   private final YoFrameVector3D yoCurrentAcceleration;

   private final YoFrameQuaternion yoInitialOrientation;
   private final YoFrameQuaternion yoFinalOrientation;

   private final YoFrameQuaternion yoCurrentOrientation;
   private final YoFrameVector3D yoCurrentAngularVelocity;
   private final YoFrameVector3D yoCurrentAngularAcceleration;

   private final YoFramePoint3D yoInitialPositionWorld;
   private final YoFramePoint3D yoFinalPositionWorld;
   private final YoFramePoint3D yoCurrentPositionWorld;
   private final YoFramePoint3D yoCurrentAdjustedPositionWorld;

   private boolean rotateHandAngleAboutAxis = false;

   private boolean visualize = true;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   private final YoFramePoint3D circleOrigin;
   private final YoFrameVector3D rotationAxis;
   private final ReferenceFrame circleFrame;
   private final ReferenceFrame trajectoryFrame;
   private ReferenceFrame controlledFrame;
   private final PoseReferenceFrame tangentialCircleFrame;
   private final FramePose3D tangentialCircleFramePose = new FramePose3D();
   private final YoFramePoseUsingYawPitchRoll yoTangentialCircleFramePose;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   public CirclePoseTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, DoubleProvider trajectoryTimeProvider,
         YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.desiredTrajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "Time", registry);

      //      this.anglePolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 2, registry);
      this.anglePolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 4, registry);
      //      this.anglePolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);

      this.desiredTrajectoryTimeProvider = trajectoryTimeProvider;

      this.trajectoryFrame = trajectoryFrame;

      initialRadius = new YoDouble(namePrefix + "Radius", registry);
      initialZ = new YoDouble(namePrefix + "ZPosition", registry);
      initialAngle = new YoDouble(namePrefix + "InitialAngle", registry);
      finalAngle = new YoDouble(namePrefix + "FinalAngle", registry);

      isCurrentAngleBeingAdjusted = new YoBoolean(namePrefix + "IsCurrentAngleBeingAdjusted", registry);
      maximumAngleTrackingErrorTolerated = new YoDouble(namePrefix + "MaxAngleTrackingErrorTolerated", registry);
      maximumAngleTrackingErrorTolerated.set(Math.toRadians(30.0));
      currentControlledFrameRelativeAngle = new YoDouble(namePrefix + "CurrentControlledFrameAngle", registry);
      currentAngleTrackingError = new YoDouble(namePrefix + "CurrentAngleTrackingError", registry);
      currentAdjustedRelativeAngle = new YoDouble(namePrefix + "CurrentAdjustedRelativeAngle", registry);

      desiredRotationAngle = new YoDouble(namePrefix + "DesiredRotationAngle", registry);
      currentRelativeAngle = new YoDouble(namePrefix + "CurrentRelativeAngle", registry);

      yoInitialPosition = new YoFramePoint3D(namePrefix + "InitialPosition", trajectoryFrame, registry);
      yoFinalPosition = new YoFramePoint3D(namePrefix + "FinalPosition", trajectoryFrame, registry);

      yoCurrentPosition = new YoFramePoint3D(namePrefix + "CurrentPosition", trajectoryFrame, registry);
      yoCurrentVelocity = new YoFrameVector3D(namePrefix + "CurrentVelocity", trajectoryFrame, registry);
      yoCurrentAcceleration = new YoFrameVector3D(namePrefix + "CurrentAcceleration", trajectoryFrame, registry);

      yoInitialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", trajectoryFrame, registry);
      yoFinalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", trajectoryFrame, registry);

      yoCurrentOrientation = new YoFrameQuaternion(namePrefix + "CurrentOrientation", trajectoryFrame, registry);
      yoCurrentAngularVelocity = new YoFrameVector3D(namePrefix + "CurrentAngularVelocity", trajectoryFrame, registry);
      yoCurrentAngularAcceleration = new YoFrameVector3D(namePrefix + "CurrentAngularAcceleration", trajectoryFrame, registry);

      yoInitialPositionWorld = new YoFramePoint3D(namePrefix + "InitialPositionWorld", worldFrame, registry);
      yoFinalPositionWorld = new YoFramePoint3D(namePrefix + "FinalPositionWorld", worldFrame, registry);
      yoCurrentPositionWorld = new YoFramePoint3D(namePrefix + "CurrentPositionWorld", worldFrame, registry);
      yoCurrentAdjustedPositionWorld = new YoFramePoint3D(namePrefix + "CurrentAdjustedPositionWorld", worldFrame, registry);

      circleOrigin = new YoFramePoint3D(namePrefix + "CircleOrigin", trajectoryFrame, registry);
      rotationAxis = new YoFrameVector3D(namePrefix + "RotationAxis", trajectoryFrame, registry);
      rotationAxis.set(0.0, 0.0, 1.0);

      circleFrame = new ReferenceFrame("CircleFrame", trajectoryFrame)
      {
         private final Vector3D localTranslation = new Vector3D();
         private final Vector3D localRotationAxis = new Vector3D();
         private final AxisAngle localAxisAngle = new AxisAngle();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localTranslation.set(circleOrigin);
            localRotationAxis.set(rotationAxis);
            EuclidGeometryTools.orientation3DFromZUpToVector3D(localRotationAxis, localAxisAngle);
            transformToParent.set(localAxisAngle, localTranslation);
         }
      };

      tangentialCircleFrame = new PoseReferenceFrame("TangentialCircleFrame", circleFrame);
      yoTangentialCircleFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "TangentialCircleFramePose", worldFrame, registry);

      if (this.visualize && yoGraphicsListRegistry != null)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", yoCurrentPositionWorld, 0.025, YoAppearance.Blue());
         final YoGraphicPosition currentAdjustedPositionViz = new YoGraphicPosition(namePrefix + "CurrentAdjustedPosition", yoCurrentAdjustedPositionWorld,
               0.023, YoAppearance.Gold());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", yoInitialPositionWorld, 0.02,
               YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", yoFinalPositionWorld, 0.02, YoAppearance.Red());
         final YoGraphicCoordinateSystem tangentialFrameViz = new YoGraphicCoordinateSystem(namePrefix + "TangentialFrame", yoTangentialCircleFramePose, 0.2,
               YoAppearance.Pink());

         YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix + "CircleTraj");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(currentAdjustedPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(tangentialFrameViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowViz", registry);
         showViz.addListener(new YoVariableChangedListener()
         {
            public void changed(YoVariable v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               currentAdjustedPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               tangentialFrameViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
            }
         });
         showViz.notifyListeners();
      }
      else
      {
         visualize = false;
         bagOfBalls = null;
         showViz = null;
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Set rotation axis in trajectoryFrame.  Default axis is zUp in trajectoryFrame.
    * 
    * @param circleCenterInTrajectoryFrame
    * @param circleNormalInTrajectoryFrame
    */
   public void updateCircleFrame(Point3D circleCenterInTrajectoryFrame, Vector3D circleNormalInTrajectoryFrame)
   {
      circleOrigin.set(circleCenterInTrajectoryFrame);
      rotationAxis.set(circleNormalInTrajectoryFrame);
      rotationAxis.normalize();
      circleFrame.update();
   }

   public void updateCircleFrame(FramePoint3D circleCenter, FrameVector3D circleNormal)
   {
      circleOrigin.setMatchingFrame(circleCenter);
      rotationAxis.setMatchingFrame(circleNormal);
      rotationAxis.normalize();
      circleFrame.update();
   }

   public void setMaximumAngleTrackingErrorTolerated(double maximumAngle)
   {
      this.maximumAngleTrackingErrorTolerated.set(maximumAngle);
   }

   public void setControlledFrame(ReferenceFrame controlledFrame)
   {
      this.controlledFrame = controlledFrame;
   }

   public void setDesiredRotationAngle(double desiredRotationAngle)
   {
      this.desiredRotationAngle.set(desiredRotationAngle);
   }

   public void setInitialPoseToMatchReferenceFrame(ReferenceFrame referenceFrame)
   {
      initialPosition.setToZero(referenceFrame);
      initialOrientation.setToZero(referenceFrame);
      initialPosition.changeFrame(trajectoryFrame);
      initialOrientation.changeFrame(trajectoryFrame);
      yoInitialPosition.set(initialPosition);
      yoInitialOrientation.set(initialOrientation);
   }

   public void setInitialPose(FramePose3D initialPose)
   {
      initialPose.changeFrame(trajectoryFrame);
      initialPose.get(initialPosition, initialOrientation);
      yoInitialPosition.set(initialPosition);
      yoInitialOrientation.set(initialOrientation);
   }

   public void setControlHandAngleAboutAxis(boolean controlIfTrue)
   {
      rotateHandAngleAboutAxis = controlIfTrue;
   }

   public void initialize()
   {
      currentTime.set(0.0);
      desiredTrajectoryTime.set(desiredTrajectoryTimeProvider.getValue());

      initialPosition.setIncludingFrame(yoInitialPosition);
      initialPosition.changeFrame(circleFrame);

      if (rotateHandAngleAboutAxis)
         rotateInitialOrientation(finalOrientation, desiredRotationAngle.getDoubleValue());
      else
         finalOrientation.setIncludingFrame(initialOrientation);
      finalOrientation.changeFrame(trajectoryFrame);
      yoFinalOrientation.set(finalOrientation);

      double x = initialPosition.getX();
      double y = initialPosition.getY();
      initialZ.set(initialPosition.getZ());

      initialRadius.set(Math.sqrt(x * x + y * y));
      initialAngle.set(Math.atan2(y, x));
      finalAngle.set(initialAngle.getDoubleValue() + desiredRotationAngle.getDoubleValue());

      //      anglePolynomial.setLinear(0.0, desiredTrajectoryTime.getDoubleValue(), initialAngle.getDoubleValue(), finalAngle.getDoubleValue());
      anglePolynomial.setCubic(0.0, desiredTrajectoryTime.getDoubleValue(), initialAngle.getDoubleValue(), 0.0, finalAngle.getDoubleValue(), 0.0);
      //      anglePolynomial.setQuintic(0.0, desiredTrajectoryTime.getDoubleValue(), initialAngle.getDoubleValue(), 0.0, 0.0, finalAngle.getDoubleValue(), 0.0, 0.0);

      double xFinal = initialRadius.getDoubleValue() * Math.cos(finalAngle.getDoubleValue());
      double yFinal = initialRadius.getDoubleValue() * Math.sin(finalAngle.getDoubleValue());
      double zFinal = initialZ.getDoubleValue();
      finalPosition.setIncludingFrame(circleFrame, xFinal, yFinal, zFinal);
      yoFinalPosition.setMatchingFrame(finalPosition);

      rotateInitialOrientation(finalOrientation, finalAngle.getDoubleValue());

      currentAngleTrackingError.set(0.0);
      currentControlledFrameRelativeAngle.set(initialAngle.getDoubleValue());

      updateTangentialCircleFrame();

      if (visualize)
         visualizeTrajectory();
   }

   public void compute(double time)
   {
      compute(time, true);
   }

   public void compute(double time, boolean adjustAngle)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, desiredTrajectoryTime.getDoubleValue());
      anglePolynomial.compute(time);

      double angle = anglePolynomial.getPosition();
      double angleDot = anglePolynomial.getVelocity();
      double angleDDot = anglePolynomial.getAcceleration();

      double cos = Math.cos(angle);
      double sin = Math.sin(angle);
      double r = initialRadius.getDoubleValue();

      double x = r * cos;
      double y = r * sin;
      double z = initialZ.getDoubleValue();

      currentPosition.setIncludingFrame(circleFrame, x, y, z);
      yoCurrentPositionWorld.setMatchingFrame(currentPosition);

      currentRelativeAngle.set(computeAngleDifferenceMinusPiToPi(angle, initialAngle.getDoubleValue()));
      if (adjustAngle)
         currentAdjustedRelativeAngle.set(adjustCurrentDesiredRelativeAngle(currentRelativeAngle.getDoubleValue()));
      else
         currentAdjustedRelativeAngle.set(currentRelativeAngle.getDoubleValue());

      angle = trimAngleMinusPiToPi(currentAdjustedRelativeAngle.getDoubleValue() + initialAngle.getDoubleValue());

      if (isDone())
      {
         angle = finalAngle.getDoubleValue();
         angleDot = 0.0;
         angleDDot = 0.0;
      }

      cos = Math.cos(angle);
      sin = Math.sin(angle);

      x = r * cos;
      y = r * sin;

      double xDot = -r * sin * angleDot;
      double yDot = x * angleDot;
      double zDot = 0.0;

      double xDDot = -r * cos * angleDot * angleDot - y * angleDDot;
      double yDDot = xDot * angleDot + x * angleDDot;
      double zDDot = 0.0;

      currentPosition.setIncludingFrame(circleFrame, x, y, z);
      currentVelocity.setIncludingFrame(circleFrame, xDot, yDot, zDot);
      currentAcceleration.setIncludingFrame(circleFrame, xDDot, yDDot, zDDot);

      if (rotateHandAngleAboutAxis)
      {
         rotateInitialOrientation(currentOrientation, angle - initialAngle.getDoubleValue());
         currentAngularVelocity.setIncludingFrame(circleFrame, 0.0, 0.0, angleDot);
         currentAngularAcceleration.setIncludingFrame(circleFrame, 0.0, 0.0, angleDDot);
      }
      else
      {
         currentOrientation.setIncludingFrame(initialOrientation);
         currentAngularVelocity.setIncludingFrame(circleFrame, 0.0, 0.0, 0.0);
         currentAngularAcceleration.setIncludingFrame(circleFrame, 0.0, 0.0, 0.0);
      }

      yoCurrentPosition.setMatchingFrame(currentPosition);
      yoCurrentAdjustedPositionWorld.setMatchingFrame(currentPosition);
      yoCurrentVelocity.setMatchingFrame(currentVelocity);
      yoCurrentAcceleration.setMatchingFrame(currentAcceleration);

      currentOrientation.changeFrame(trajectoryFrame);
      yoCurrentOrientation.set(currentOrientation);
      yoCurrentAngularVelocity.setMatchingFrame(currentAngularVelocity);
      yoCurrentAngularAcceleration.setMatchingFrame(currentAngularAcceleration);

      updateTangentialCircleFrame();
   }

   private void updateTangentialCircleFrame()
   {
      if (controlledFrame != null)
      {
         tangentialCircleFramePose.setToZero(controlledFrame);
      }
      else
      {
         tangentialCircleFramePose.setToZero(currentPosition.getReferenceFrame());
         tangentialCircleFramePose.getPosition().set(currentPosition);
      }

      tangentialCircleFramePose.changeFrame(circleFrame);

      double x = tangentialCircleFramePose.getX();
      double y = tangentialCircleFramePose.getY();

      double yaw = trimAngleMinusPiToPi(Math.PI / 2.0 + Math.atan2(y, x));
      tangentialCircleFramePose.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
      tangentialCircleFrame.setPoseAndUpdate(tangentialCircleFramePose);
      yoTangentialCircleFramePose.setMatchingFrame(tangentialCircleFramePose);
   }

   private final FramePoint3D currentControlledFramePosition = new FramePoint3D();

   private double adjustCurrentDesiredRelativeAngle(double currentDesiredRelativeAngle)
   {
      if (controlledFrame == null)
      {
         isCurrentAngleBeingAdjusted.set(false);
         return currentDesiredRelativeAngle;
      }

      currentControlledFramePosition.setToZero(controlledFrame);
      currentControlledFramePosition.changeFrame(circleFrame);

      double x = currentControlledFramePosition.getX();
      double y = currentControlledFramePosition.getY();

      currentControlledFrameRelativeAngle.set(computeAngleDifferenceMinusPiToPi(Math.atan2(y, x), initialAngle.getDoubleValue()));

      currentAngleTrackingError.set(computeAngleDifferenceMinusPiToPi(currentDesiredRelativeAngle, currentControlledFrameRelativeAngle.getDoubleValue()));

      if (computeAngleDifferenceMinusPiToPi(currentAngleTrackingError.getDoubleValue(), maximumAngleTrackingErrorTolerated.getDoubleValue()) > 0.0)
      {
         isCurrentAngleBeingAdjusted.set(true);
         return trimAngleMinusPiToPi(currentControlledFrameRelativeAngle.getDoubleValue() + maximumAngleTrackingErrorTolerated.getDoubleValue());
      }
      else if (trimAngleMinusPiToPi(currentAngleTrackingError.getDoubleValue() + maximumAngleTrackingErrorTolerated.getDoubleValue()) < 0.0)
      {
         isCurrentAngleBeingAdjusted.set(true);
         return computeAngleDifferenceMinusPiToPi(currentControlledFrameRelativeAngle.getDoubleValue(), maximumAngleTrackingErrorTolerated.getDoubleValue());
      }
      else
      {
         isCurrentAngleBeingAdjusted.set(false);
         return currentDesiredRelativeAngle;
      }
   }

   private final RigidBodyTransform axisRotationTransform = new RigidBodyTransform();

   private void rotateInitialOrientation(FrameQuaternion orientationToPack, double angleFromInitialOrientation)
   {
      initialOrientation.changeFrame(circleFrame);
      orientationToPack.setIncludingFrame(initialOrientation);

      axisRotationTransform.setRotationYawAndZeroTranslation(angleFromInitialOrientation);

      orientationToPack.applyTransform(axisRotationTransform);
   }

   private void visualizeTrajectory()
   {
      initialPosition.setIncludingFrame(yoInitialPosition);
      yoInitialPositionWorld.setMatchingFrame(initialPosition);
      finalPosition.setIncludingFrame(yoFinalPosition);
      yoFinalPositionWorld.setMatchingFrame(finalPosition);

      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = (double) i / ((double) numberOfBalls - 1) * desiredTrajectoryTime.getDoubleValue();
         compute(t, false);
         ballPosition.setIncludingFrame(yoCurrentPosition);
         ballPosition.changeFrame(worldFrame);
         bagOfBalls.setBallLoop(ballPosition);
      }
      reset();
   }

   private void reset()
   {
      compute(0.0);
   }

   public double getCurrentAngularDisplacement()
   {
      return currentRelativeAngle.getDoubleValue();
   }

   public ReferenceFrame getTangentialCircleFrame()
   {
      return tangentialCircleFrame;
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= desiredTrajectoryTime.getDoubleValue() && !isCurrentAngleBeingAdjusted.getBooleanValue();
   }

   public ReferenceFrame getCircleFrame()
   {
      return circleFrame;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(yoCurrentAdjustedPositionWorld);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(yoCurrentVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(yoCurrentAcceleration);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(yoCurrentOrientation);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(yoCurrentAngularVelocity);
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(yoCurrentAngularAcceleration);
   }

   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(yoCurrentAdjustedPositionWorld, yoCurrentOrientation);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   public void showVisualization()
   {
      if (!visualize)
         return;

      showViz.set(true);
   }

   public void hideVisualization()
   {
      if (!visualize)
         return;

      showViz.set(false);
   }
}