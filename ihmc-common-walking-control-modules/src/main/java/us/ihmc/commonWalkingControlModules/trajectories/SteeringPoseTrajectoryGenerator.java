package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.geometry.AngleTools.computeAngleDifferenceMinusPiToPi;
import static us.ihmc.robotics.geometry.AngleTools.trimAngleMinusPiToPi;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class SteeringPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoDouble currentTime;
   private final YoPolynomial steeringAnglePolynomial;

   private final YoDouble trajectoryTime;
   private final YoDouble desiredSteeringSpeed;
   private final YoDouble steeringWheelRadius;
   private final YoDouble initialZ;

   private final YoDouble initialSteeringAngle;
   private final YoDouble currentRelativeSteeringAngle;
   private final YoDouble finalSteeringAngle;

   private final YoBoolean isCurrentAngleBeingAdjusted;
   private final YoDouble maximumAngleTrackingErrorTolerated;
   private final YoDouble currentControlledFrameRelativeAngle;
   private final YoDouble currentAngleTrackingError;
   private final YoDouble currentAdjustedRelativeAngle;

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

   private boolean visualize = true;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   private final YoFramePoint3D steeringWheelCenter;
   private final YoFrameVector3D steeringWheelRotationAxis;
   private final YoFrameVector3D steeringWheelZeroAxis;
   private final ReferenceFrame steeringWheelFrame;
   private final FramePose3D steeringWheelFramePose = new FramePose3D();
   private final YoFramePoseUsingYawPitchRoll yoSteeringWheelFramePose;
   private final ReferenceFrame trajectoryFrame;
   private ReferenceFrame controlledFrame;
   private final PoseReferenceFrame tangentialSteeringFrame;
   private final FramePose3D tangentialSteeringFramePose = new FramePose3D();
   private final YoFramePoseUsingYawPitchRoll yoTangentialSteeringFramePose;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   public SteeringPoseTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new YoDouble(namePrefix + "SteeringTrajectoryTime", registry);
      this.desiredSteeringSpeed = new YoDouble(namePrefix + "DesiredSteeringSpeed", registry);
      this.currentTime = new YoDouble(namePrefix + "Time", registry);

      this.steeringAnglePolynomial = new YoPolynomial(namePrefix + "SteeringParameterPolynomial", 2, registry);

      this.trajectoryFrame = trajectoryFrame;

      initialZ = new YoDouble(namePrefix + "SteeringZPosition", registry);

      isCurrentAngleBeingAdjusted = new YoBoolean(namePrefix + "IsCurrentSteeringAngleBeingAdjusted", registry);
      maximumAngleTrackingErrorTolerated = new YoDouble(namePrefix + "MaxSteeringAngleTrackingErrorTolerated", registry);
      maximumAngleTrackingErrorTolerated.set(Math.toRadians(30.0));
      currentControlledFrameRelativeAngle = new YoDouble(namePrefix + "CurrentControlledFrameSteeringAngle", registry);
      currentAngleTrackingError = new YoDouble(namePrefix + "CurrentSteeringAngleTrackingError", registry);
      currentAdjustedRelativeAngle = new YoDouble(namePrefix + "CurrentAdjustedRelativeSteeringAngle", registry);

      steeringWheelRadius = new YoDouble(namePrefix + "SteeringWheelRadius", registry);

      initialSteeringAngle = new YoDouble(namePrefix + "InitialSteeringAngle", registry);
      currentRelativeSteeringAngle = new YoDouble(namePrefix + "CurrentRelativeSteeringAngle", registry);
      finalSteeringAngle = new YoDouble(namePrefix + "FinalSteeringAngle", registry);

      yoInitialPosition = new YoFramePoint3D(namePrefix + "InitialSteeringPosition", trajectoryFrame, registry);
      yoFinalPosition = new YoFramePoint3D(namePrefix + "FinalSteeringPosition", trajectoryFrame, registry);

      yoCurrentPosition = new YoFramePoint3D(namePrefix + "CurrentSteeringPosition", trajectoryFrame, registry);
      yoCurrentVelocity = new YoFrameVector3D(namePrefix + "CurrentSteeringVelocity", trajectoryFrame, registry);
      yoCurrentAcceleration = new YoFrameVector3D(namePrefix + "CurrentSteeringAcceleration", trajectoryFrame, registry);

      yoInitialOrientation = new YoFrameQuaternion(namePrefix + "InitialSteeringOrientation", trajectoryFrame, registry);
      yoFinalOrientation = new YoFrameQuaternion(namePrefix + "FinalSteeringOrientation", trajectoryFrame, registry);

      yoCurrentOrientation = new YoFrameQuaternion(namePrefix + "CurrentSteeringOrientation", trajectoryFrame, registry);
      yoCurrentAngularVelocity = new YoFrameVector3D(namePrefix + "CurrentSteeringAngularVelocity", trajectoryFrame, registry);
      yoCurrentAngularAcceleration = new YoFrameVector3D(namePrefix + "CurrentSteeringAngularAcceleration", trajectoryFrame, registry);

      yoInitialPositionWorld = new YoFramePoint3D(namePrefix + "InitialSteeringPositionWorld", worldFrame, registry);
      yoFinalPositionWorld = new YoFramePoint3D(namePrefix + "FinalSteeringPositionWorld", worldFrame, registry);
      yoCurrentPositionWorld = new YoFramePoint3D(namePrefix + "CurrentSteeringPositionWorld", worldFrame, registry);
      yoCurrentAdjustedPositionWorld = new YoFramePoint3D(namePrefix + "CurrentAdjustedSteeringPositionWorld", worldFrame, registry);

      steeringWheelCenter = new YoFramePoint3D(namePrefix + "SteeringWheelCenter", trajectoryFrame, registry);
      steeringWheelRotationAxis = new YoFrameVector3D(namePrefix + "SteeringWheelRotationAxis", trajectoryFrame, registry);
      steeringWheelRotationAxis.set(0.0, 0.0, 1.0);
      steeringWheelZeroAxis = new YoFrameVector3D(namePrefix + "SteeringWheelZeroAxis", trajectoryFrame, registry);
      steeringWheelZeroAxis.set(1.0, 0.0, 0.0);

      steeringWheelFrame = new ReferenceFrame("SteeringWheelFrame", trajectoryFrame)
      {
         private final Vector3D localTranslation = new Vector3D();

         private final Vector3D localXAxis = new Vector3D();
         private final Vector3D localYAxis = new Vector3D();
         private final Vector3D localZAxis = new Vector3D();

         private final RotationMatrix localRotation = new RotationMatrix();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localZAxis.set(steeringWheelRotationAxis);
            localXAxis.set(steeringWheelZeroAxis);
            localYAxis.cross(localZAxis, localXAxis);
            localYAxis.normalize();
            localXAxis.cross(localYAxis, localZAxis);
            steeringWheelZeroAxis.set(localXAxis);

            localTranslation.set(steeringWheelCenter);
            localRotation.setColumns(localXAxis, localYAxis, localZAxis);
            transformToParent.set(localRotation, localTranslation);
         }
      };

      yoSteeringWheelFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "SteeringWheelFrame", worldFrame, registry);

      tangentialSteeringFrame = new PoseReferenceFrame("TangentialSteeringFrame", steeringWheelFrame);
      yoTangentialSteeringFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "TangentialSteeringFramePose", worldFrame, registry);

      if (this.visualize && yoGraphicsListRegistry != null)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentSteeringPosition", yoCurrentPositionWorld, 0.025,
               YoAppearance.Blue());
         final YoGraphicPosition currentAdjustedPositionViz = new YoGraphicPosition(namePrefix + "CurrentAdjustedSteeringPosition",
               yoCurrentAdjustedPositionWorld, 0.023, YoAppearance.Gold());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialSteeringPosition", yoInitialPositionWorld, 0.02,
               YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalSteeringPosition", yoFinalPositionWorld, 0.02, YoAppearance.Red());
         final YoGraphicCoordinateSystem tangentialFrameViz = new YoGraphicCoordinateSystem(namePrefix + "TangentialSteeringFrame",
               yoTangentialSteeringFramePose, 0.2, YoAppearance.Pink());
         final YoGraphicCoordinateSystem steeringWheelFrameViz = new YoGraphicCoordinateSystem(namePrefix + "SteeringWheelFrame", yoSteeringWheelFramePose, 0.2,
               YoAppearance.Green());

         YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix + "SteeringTrajectory");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(currentAdjustedPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(tangentialFrameViz);
         yoGraphicsList.add(steeringWheelFrameViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowSteeringViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               currentAdjustedPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               tangentialFrameViz.setVisible(visible);
               steeringWheelFrameViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         visualize = false;
         bagOfBalls = null;
         showViz = null;
      }

      parentRegistry.addChild(registry);
   }

   public void updateSteeringWheel(FramePoint3D center, FrameVector3D rotationAxis, FrameVector3D zeroAxis)
   {
      steeringWheelCenter.setMatchingFrame(center);
      steeringWheelRotationAxis.setMatchingFrame(rotationAxis);
      steeringWheelRotationAxis.normalize();
      steeringWheelZeroAxis.setMatchingFrame(zeroAxis);
      steeringWheelZeroAxis.normalize();
      steeringWheelFrame.update();

      steeringWheelFramePose.setToZero(steeringWheelFrame);
      steeringWheelFramePose.changeFrame(worldFrame);
      yoSteeringWheelFramePose.set(steeringWheelFramePose);
   }

   public void setSteeringWheelRadius(double radius)
   {
      steeringWheelRadius.set(radius);
   }

   public void setMaximumAngleTrackingErrorTolerated(double maximumAngle)
   {
      this.maximumAngleTrackingErrorTolerated.set(maximumAngle);
   }

   public void setDesiredSteeringSpeed(double desiredSteeringSpeed)
   {
      this.desiredSteeringSpeed.set(desiredSteeringSpeed);
   }

   public void setControlledFrame(ReferenceFrame controlledFrame)
   {
      this.controlledFrame = controlledFrame;
   }

   public void setFinalSteeringAngle(double finalSteeringAngle)
   {
      this.finalSteeringAngle.set(trimAngleMinusPiToPi(finalSteeringAngle));
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
      initialPose.get(initialPosition, initialOrientation);
      initialPosition.changeFrame(trajectoryFrame);
      initialOrientation.changeFrame(trajectoryFrame);
      yoInitialPosition.set(initialPosition);
      yoInitialOrientation.set(initialOrientation);
   }

   public void initialize()
   {
      currentTime.set(0.0);

      initialPosition.setIncludingFrame(yoInitialPosition);
      initialPosition.changeFrame(steeringWheelFrame);

      finalOrientation.setIncludingFrame(initialOrientation);
      finalOrientation.changeFrame(trajectoryFrame);
      yoFinalOrientation.set(finalOrientation);

      double x = initialPosition.getX();
      double y = initialPosition.getY();
      initialZ.set(initialPosition.getZ());

      initialSteeringAngle.set(Math.atan2(y, x));
      double initialToFinalAngle = computeAngleDifferenceMinusPiToPi(finalSteeringAngle.getDoubleValue(), initialSteeringAngle.getDoubleValue());
      double finalSteeringAngleForShortestPath = initialSteeringAngle.getDoubleValue() + initialToFinalAngle;
      trajectoryTime.set(MathTools.clamp(Math.abs(initialToFinalAngle) / desiredSteeringSpeed.getDoubleValue(), 0.25, Double.POSITIVE_INFINITY));

      steeringAnglePolynomial.setLinear(0.0, trajectoryTime.getDoubleValue(), initialSteeringAngle.getDoubleValue(), finalSteeringAngleForShortestPath);

      double xFinal = steeringWheelRadius.getDoubleValue() * Math.cos(finalSteeringAngle.getDoubleValue());
      double yFinal = steeringWheelRadius.getDoubleValue() * Math.sin(finalSteeringAngle.getDoubleValue());
      double zFinal = initialZ.getDoubleValue();
      finalPosition.setIncludingFrame(steeringWheelFrame, xFinal, yFinal, zFinal);
      yoFinalPosition.setMatchingFrame(finalPosition);

      currentAngleTrackingError.set(0.0);
      currentControlledFrameRelativeAngle.set(initialSteeringAngle.getDoubleValue());

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
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      steeringAnglePolynomial.compute(time);

      double angle = steeringAnglePolynomial.getPosition();
      double angleDot = steeringAnglePolynomial.getVelocity();
      double angleDDot = steeringAnglePolynomial.getAcceleration();

      double cos = Math.cos(angle);
      double sin = Math.sin(angle);
      double r = steeringWheelRadius.getDoubleValue();

      double x = r * cos;
      double y = r * sin;
      double z = initialZ.getDoubleValue();

      currentPosition.setIncludingFrame(steeringWheelFrame, x, y, z);
      yoCurrentPositionWorld.setMatchingFrame(currentPosition);

      currentRelativeSteeringAngle.set(computeAngleDifferenceMinusPiToPi(angle, initialSteeringAngle.getDoubleValue()));
      if (adjustAngle)
         currentAdjustedRelativeAngle.set(adjustCurrentDesiredRelativeAngle(currentRelativeSteeringAngle.getDoubleValue()));
      else
         currentAdjustedRelativeAngle.set(currentRelativeSteeringAngle.getDoubleValue());

      angle = trimAngleMinusPiToPi(currentAdjustedRelativeAngle.getDoubleValue() + initialSteeringAngle.getDoubleValue());

      if (isDone())
      {
         angle = finalSteeringAngle.getDoubleValue();
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

      currentPosition.setIncludingFrame(steeringWheelFrame, x, y, z);
      currentVelocity.setIncludingFrame(steeringWheelFrame, xDot, yDot, zDot);
      currentAcceleration.setIncludingFrame(steeringWheelFrame, xDDot, yDDot, zDDot);

      currentOrientation.setIncludingFrame(initialOrientation);
      currentAngularVelocity.setIncludingFrame(steeringWheelFrame, 0.0, 0.0, 0.0);
      currentAngularAcceleration.setIncludingFrame(steeringWheelFrame, 0.0, 0.0, 0.0);

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
         tangentialSteeringFramePose.setToZero(controlledFrame);
      }
      else
      {
         tangentialSteeringFramePose.setToZero(currentPosition.getReferenceFrame());
         tangentialSteeringFramePose.setPosition(currentPosition);
      }

      tangentialSteeringFramePose.changeFrame(steeringWheelFrame);

      double x = tangentialSteeringFramePose.getX();
      double y = tangentialSteeringFramePose.getY();

      double yaw = trimAngleMinusPiToPi(Math.PI / 2.0 + Math.atan2(y, x));
      tangentialSteeringFramePose.setOrientationYawPitchRoll(yaw, 0.0, 0.0);
      tangentialSteeringFrame.setPoseAndUpdate(tangentialSteeringFramePose);
      yoTangentialSteeringFramePose.setMatchingFrame(tangentialSteeringFramePose);
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
      currentControlledFramePosition.changeFrame(steeringWheelFrame);

      double x = currentControlledFramePosition.getX();
      double y = currentControlledFramePosition.getY();

      currentControlledFrameRelativeAngle.set(computeAngleDifferenceMinusPiToPi(Math.atan2(y, x), initialSteeringAngle.getDoubleValue()));

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

   private void visualizeTrajectory()
   {
      initialPosition.setIncludingFrame(yoInitialPosition);
      yoInitialPositionWorld.setMatchingFrame(initialPosition);
      finalPosition.setIncludingFrame(yoFinalPosition);
      yoFinalPositionWorld.setMatchingFrame(finalPosition);

      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = (double) i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
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
      return currentRelativeSteeringAngle.getDoubleValue();
   }

   public ReferenceFrame getTangentialSteeringFrame()
   {
      return tangentialSteeringFrame;
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue() && !isCurrentAngleBeingAdjusted.getBooleanValue();
   }

   public ReferenceFrame getSteeringWheelFrame()
   {
      return steeringWheelFrame;
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