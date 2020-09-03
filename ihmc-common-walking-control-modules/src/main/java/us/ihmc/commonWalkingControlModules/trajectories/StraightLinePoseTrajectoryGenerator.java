package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.euclid.referenceFrame.*;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class StraightLinePoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final YoRegistry registry;

   private final YoMutableFramePoint3D initialPosition;
   private final YoMutableFramePoint3D finalPosition;

   private final YoMutableFramePoint3D currentPosition;
   private final YoMutableFrameVector3D currentVelocity;
   private final YoMutableFrameVector3D currentAcceleration;

   private final YoMutableFrameQuaternion initialOrientation;
   private final YoMutableFrameQuaternion finalOrientation;
   private final YoFrameYawPitchRoll initialOrientationForViz;
   private final YoFrameYawPitchRoll finalOrientationForViz;

   private final YoMutableFrameQuaternion currentOrientation;
   private final YoMutableFrameVector3D currentAngularVelocity;
   private final YoMutableFrameVector3D currentAngularAcceleration;

   // (Sylvain) I created 2 YoPolynomial to match the StraightLinePositionTrajectoryGenerator and OrientationInterpolationTrajectoryGenerator.
   // Not sure if that's actually necessary, the cubic one maybe enough.
   private final YoPolynomial quinticParameterPolynomial;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   // For viz
   private final boolean visualize;
   private final YoGraphicsList yoGraphicsList;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   private YoFrameYawPitchRoll currentOrientationForViz;

   private final List<ImmutablePair<FramePoint3DReadOnly, YoFramePoint3D>> visualizationUpdatables = new ArrayList<>();

   public StraightLinePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, referenceFrame, parentRegistry, false, null);
   }

   public StraightLinePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry, boolean visualize,
                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      initialPosition = new YoMutableFramePoint3D(namePrefix + "InitialPosition", "", registry, referenceFrame);
      finalPosition = new YoMutableFramePoint3D(namePrefix + "FinalPosition", "", registry, referenceFrame);

      currentPosition = new YoMutableFramePoint3D(namePrefix + "CurrentPosition", "", registry, referenceFrame);
      currentVelocity = new YoMutableFrameVector3D(namePrefix + "CurrentVelocity", "", registry, referenceFrame);
      currentAcceleration = new YoMutableFrameVector3D(namePrefix + "CurrentAcceleration", "", registry, referenceFrame);

      initialOrientation = new YoMutableFrameQuaternion(namePrefix + "InitialOrientation", "", registry, referenceFrame);
      finalOrientation = new YoMutableFrameQuaternion(namePrefix + "FinalOrientation", "", registry, referenceFrame);
      initialOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "InitialOrientationForViz", ReferenceFrame.getWorldFrame(), registry);
      finalOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "FinalOrientationForViz", ReferenceFrame.getWorldFrame(), registry);
      currentOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "CurrentOrientationForViz", ReferenceFrame.getWorldFrame(), registry);

      currentOrientation = new YoMutableFrameQuaternion(namePrefix + "CurrentOrientation", "", registry, referenceFrame);
      currentAngularVelocity = new YoMutableFrameVector3D(namePrefix + "CurrentAngularVelocity", "", registry, referenceFrame);
      currentAngularAcceleration = new YoMutableFrameVector3D(namePrefix + "CurrentAngularAcceleration", "", registry, referenceFrame);

      quinticParameterPolynomial = new YoPolynomial(namePrefix + "QuinticParameterPolynomial", 6, registry);

      currentTime = new YoDouble(namePrefix + "Time", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         YoFramePoint3D currentPositionInWorld = new YoFramePoint3D(namePrefix + "CurrentPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<FramePoint3DReadOnly, YoFramePoint3D>(currentPosition, currentPositionInWorld));
         YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPositionInWorld, 0.025, YoAppearance.Blue());

         YoFramePoint3D initialPositionInWorld = new YoFramePoint3D(namePrefix + "InitialPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<FramePoint3DReadOnly, YoFramePoint3D>(initialPosition, initialPositionInWorld));
         YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPositionInWorld, 0.02, YoAppearance.BlueViolet());

         YoFramePoint3D finalPositionInWorld = new YoFramePoint3D(namePrefix + "FinalPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<FramePoint3DReadOnly, YoFramePoint3D>(finalPosition, finalPositionInWorld));
         YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPositionInWorld, 0.02, YoAppearance.Red());

         YoGraphicCoordinateSystem initialPoseViz = new YoGraphicCoordinateSystem(namePrefix + "InitialPose", initialPositionInWorld, initialOrientationForViz,
                                                                                  0.1);
         YoGraphicCoordinateSystem finalPoseViz = new YoGraphicCoordinateSystem(namePrefix + "FinalPose", finalPositionInWorld, finalOrientationForViz, 0.1);
         YoGraphicCoordinateSystem currentPoseViz = new YoGraphicCoordinateSystem(namePrefix + "CurrentPose", currentPositionInWorld, currentOrientationForViz,
                                                                                  0.25);

         yoGraphicsList = new YoGraphicsList(namePrefix + "StraightLineTrajectory");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(initialPoseViz);
         yoGraphicsList.add(finalPoseViz);
         yoGraphicsList.add(currentPoseViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowViz", registry);
         showViz.addListener(new YoVariableChangedListener()
         {
            @Override
            public void changed(YoVariable v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
            }
         });
         showViz.notifyListeners();
      }
      else
      {
         yoGraphicsList = null;
         bagOfBalls = null;
         showViz = null;
      }
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      initialPosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);
      currentPosition.changeFrame(referenceFrame);
      currentVelocity.changeFrame(referenceFrame);
      currentAcceleration.changeFrame(referenceFrame);

      initialOrientation.changeFrame(referenceFrame);
      finalOrientation.changeFrame(referenceFrame);
      currentOrientation.changeFrame(referenceFrame);
      currentAngularVelocity.changeFrame(referenceFrame);
      currentAngularAcceleration.changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      initialPosition.setToZero(referenceFrame);
      finalPosition.setToZero(referenceFrame);
      currentPosition.setToZero(referenceFrame);
      currentVelocity.setToZero(referenceFrame);
      currentAcceleration.setToZero(referenceFrame);

      initialOrientation.setToZero(referenceFrame);
      finalOrientation.setToZero(referenceFrame);
      currentOrientation.setToZero(referenceFrame);
      currentAngularVelocity.setToZero(referenceFrame);
      currentAngularAcceleration.setToZero(referenceFrame);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
   }

   public void setInitialPose(FramePose3D initialPose)
   {
      initialPose.get(tempPosition, tempOrientation);

      initialPosition.setMatchingFrame(tempPosition);
      initialOrientation.setMatchingFrame(tempOrientation);

      initialOrientationForViz.setMatchingFrame(tempOrientation);
   }

   public void setInitialPose(FramePoint3D initialPosition, FrameQuaternion initialOrientation)
   {
      this.initialPosition.setMatchingFrame(initialPosition);
      this.initialOrientation.setMatchingFrame(initialOrientation);

      initialOrientationForViz.setMatchingFrame(initialOrientation);
   }

   public void setFinalPose(FramePose3D finalPose)
   {
      finalPose.get(tempPosition, tempOrientation);

      finalPosition.setMatchingFrame(tempPosition);
      finalOrientation.setMatchingFrame(tempOrientation);

      finalOrientationForViz.setMatchingFrame(tempOrientation);
   }

   public void setFinalPose(FramePoint3D finalPosition, FrameQuaternion finalOrientation)
   {
      this.finalPosition.setMatchingFrame(finalPosition);
      this.finalOrientation.setMatchingFrame(finalOrientation);

      finalOrientationForViz.setMatchingFrame(finalOrientation);

      tempPosition.setIncludingFrame(finalPosition);
      tempOrientation.setIncludingFrame(finalOrientation);
      finalOrientationForViz.setMatchingFrame(tempOrientation);
   }

   @Override
   public void initialize()
   {
      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      quinticParameterPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      reset();

      if (visualize)
         visualizeTrajectory();
   }

   private void reset()
   {
      currentTime.set(0.0);
      currentPosition.set(initialPosition);
      currentVelocity.setToZero();
      currentAcceleration.setToZero();
      currentOrientation.set(initialOrientation);
      currentAngularVelocity.setToZero();
      currentAngularAcceleration.setToZero();
   }

   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      quinticParameterPolynomial.compute(time);
      boolean isDone = isDone();
      double alphaVel = isDone ? 0.0 : quinticParameterPolynomial.getVelocity();
      double alphaAcc = isDone ? 0.0 : quinticParameterPolynomial.getAcceleration();
      double alphaAngVel = isDone ? 0.0 : quinticParameterPolynomial.getVelocity();
      double alphaAngAcc = isDone ? 0.0 : quinticParameterPolynomial.getAcceleration();

      if (isDone)
      {
         currentPosition.set(finalPosition);
         currentVelocity.setToZero();
         currentAcceleration.setToZero();

         currentOrientation.set(finalOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
      }
      else
      {
         currentPosition.interpolate(initialPosition, finalPosition, quinticParameterPolynomial.getPosition());
         currentVelocity.sub(finalPosition, initialPosition);
         currentVelocity.scale(alphaVel);
         currentAcceleration.sub(finalPosition, initialPosition);
         currentAcceleration.scale(alphaAcc);

         currentOrientation.interpolate(initialOrientation, finalOrientation, quinticParameterPolynomial.getPosition());
         orientationInterpolationCalculator.computeAngularVelocity(currentAngularVelocity, initialOrientation, finalOrientation, alphaAngVel);
         orientationInterpolationCalculator.computeAngularAcceleration(currentAngularAcceleration, initialOrientation, finalOrientation, alphaAngAcc);
      }

      tempOrientation.setIncludingFrame(currentOrientation);
      tempOrientation.changeFrame(currentOrientationForViz.getReferenceFrame());
      currentOrientationForViz.set(tempOrientation);
      for (int i = 0; i < visualizationUpdatables.size(); i++)
      {
         ImmutablePair<FramePoint3DReadOnly, YoFramePoint3D> pair = visualizationUpdatables.get(i);
         pair.getRight().setMatchingFrame(pair.getLeft());
      }
   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         ballPosition.setIncludingFrame(currentPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(ballPosition);
      }
      reset();
   }

   @Override
   public void showVisualization()
   {
      if (!visualize)
         return;

      showViz.set(true);
   }

   @Override
   public void hideVisualization()
   {
      if (!visualize)
         return;

      showViz.set(false);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(currentPosition);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAcceleration);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(currentOrientation);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(currentAngularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(currentAngularAcceleration);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.changeFrame(currentPosition.getReferenceFrame());
      framePoseToPack.getPosition().set(currentPosition);
      framePoseToPack.getOrientation().set(currentOrientation);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public ReferenceFrame getCurrentReferenceFrame()
   {
      return currentPosition.getReferenceFrame();
   }

   @Override
   public String toString()
   {
      String ret = "";
      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent position: " + currentPosition;
      ret += "\nCurrent velocity: " + currentVelocity;
      ret += "\nCurrent acceleration: " + currentAcceleration;
      ret += "\nCurrent orientation: " + currentOrientation;
      ret += "\nCurrent angular velocity: " + currentAngularVelocity;
      ret += "\nCurrent angular acceleration: " + currentAngularAcceleration;
      return ret;
   }
}
