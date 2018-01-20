package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class StraightLinePoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final boolean allowMultipleFrames;

   private final YoVariableRegistry registry;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoFrameQuaternionInMultipleFrames initialOrientation;
   private final YoFrameQuaternionInMultipleFrames finalOrientation;
   private final YoFrameOrientation initialOrientationForViz;
   private final YoFrameOrientation finalOrientationForViz;

   private final YoFrameQuaternionInMultipleFrames currentOrientation;
   private final YoFrameVectorInMultipleFrames currentAngularVelocity;
   private final YoFrameVectorInMultipleFrames currentAngularAcceleration;

   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;

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

   private YoFrameOrientation currentOrientationForViz;

   public StraightLinePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public StraightLinePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public StraightLinePoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public StraightLinePoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
         boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.allowMultipleFrames = allowMultipleFrames;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame);

      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame);

      initialOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "InitialOrientation", registry, referenceFrame);
      finalOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "FinalOrientation", registry, referenceFrame);
      initialOrientationForViz = new YoFrameOrientation(namePrefix + "InitialOrientationForViz", ReferenceFrame.getWorldFrame(), registry);
      finalOrientationForViz = new YoFrameOrientation(namePrefix + "FinalOrientationForViz", ReferenceFrame.getWorldFrame(), registry);
      currentOrientationForViz = new YoFrameOrientation(namePrefix + "CurrentOrientationForViz", ReferenceFrame.getWorldFrame(), registry);

      currentOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "CurrentOrientation", registry, referenceFrame);
      currentAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularVelocity", registry, referenceFrame);
      currentAngularAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularAcceleration", registry, referenceFrame);

      quinticParameterPolynomial = new YoPolynomial(namePrefix + "QuinticParameterPolynomial", 6, registry);

      currentTime = new YoDouble(namePrefix + "Time", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);

      multipleFramesHolders = new ArrayList<YoMultipleFramesHolder>();
      registerMultipleFramesHolders(initialPosition, finalPosition, currentPosition, currentVelocity, currentAcceleration);
      registerMultipleFramesHolders(initialOrientation, finalOrientation, currentOrientation, currentAngularVelocity, currentAngularAcceleration);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());

         final YoGraphicCoordinateSystem initialPoseViz = new YoGraphicCoordinateSystem(namePrefix + "InitialPose",
               initialPosition.buildUpdatedYoFramePointForVisualizationOnly(), initialOrientationForViz, 0.1);
         final YoGraphicCoordinateSystem finalPoseViz = new YoGraphicCoordinateSystem(namePrefix + "FinalPose",
               finalPosition.buildUpdatedYoFramePointForVisualizationOnly(), finalOrientationForViz, 0.1);
         final YoGraphicCoordinateSystem currentPoseViz = new YoGraphicCoordinateSystem(namePrefix + "CurrentPose",
               currentPosition.buildUpdatedYoFramePointForVisualizationOnly(), currentOrientationForViz, 0.25);
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
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         yoGraphicsList = null;
         bagOfBalls = null;
         showViz = null;
      }
   }

   private void registerMultipleFramesHolders(YoMultipleFramesHolder... multipleFramesHolders)
   {
      for (YoMultipleFramesHolder multipleFramesHolder : multipleFramesHolders)
         this.multipleFramesHolders.add(multipleFramesHolder);
   }

   public void registerAndSwitchFrame(ReferenceFrame desiredFrame)
   {
      registerNewTrajectoryFrame(desiredFrame);
      switchTrajectoryFrame(desiredFrame);
   }

   public void registerNewTrajectoryFrame(ReferenceFrame newReferenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).registerReferenceFrame(newReferenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      changeFrame(referenceFrame, true);
   }

   private void changeFrame(ReferenceFrame referenceFrame, boolean checkIfAllowed)
   {
      if (checkIfAllowed)
         checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).switchCurrentReferenceFrame(referenceFrame);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
   }

   public void setInitialPose(FramePose3D initialPose)
   {
      initialPose.get(tempPosition, tempOrientation);

      initialPosition.setAndMatchFrame(tempPosition);
      initialOrientation.setAndMatchFrame(tempOrientation);

      initialOrientationForViz.setAndMatchFrame(tempOrientation);
   }

   public void setInitialPose(FramePoint3D initialPosition, FrameQuaternion initialOrientation)
   {
      this.initialPosition.setAndMatchFrame(initialPosition);
      this.initialOrientation.setAndMatchFrame(initialOrientation);

      initialOrientationForViz.setAndMatchFrame(initialOrientation);
   }

   public void setFinalPose(FramePose3D finalPose)
   {
      finalPose.get(tempPosition, tempOrientation);

      finalPosition.setAndMatchFrame(tempPosition);
      finalOrientation.setAndMatchFrame(tempOrientation);

      finalOrientationForViz.setAndMatchFrame(tempOrientation);
   }

   public void setFinalPose(FramePoint3D finalPosition, FrameQuaternion finalOrientation)
   {
      this.finalPosition.setAndMatchFrame(finalPosition);
      this.finalOrientation.setAndMatchFrame(finalOrientation);

      finalOrientationForViz.setAndMatchFrame(finalOrientation);

      tempPosition.setIncludingFrame(finalPosition);
      tempOrientation.setIncludingFrame(finalOrientation);
      finalOrientationForViz.setAndMatchFrame(tempOrientation);
   }

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
   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = (double) i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         ballPosition.setIncludingFrame(currentPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(ballPosition);
      }
      reset();
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

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(currentPosition);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAcceleration);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(currentOrientation);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(currentAngularVelocity);
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(currentAngularAcceleration);
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

   public void getPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.changeFrame(currentPosition.getReferenceFrame());
      framePoseToPack.setPosition(currentPosition);
      framePoseToPack.setOrientation(currentOrientation);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public ReferenceFrame getCurrentReferenceFrame()
   {
      return currentPosition.getReferenceFrame();
   }

   private void checkIfMultipleFramesAllowed()
   {
      if (!allowMultipleFrames)
         throw new RuntimeException("Must set allowMultipleFrames to true in the constructor if you ever want to register a new frame.");
   }

   public String toString()
   {
      String ret = "";

      ReferenceFrame currentFrame = initialPosition.getReferenceFrame();

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent position: " + currentPosition.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent velocity: " + currentVelocity.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent acceleration: " + currentAcceleration.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent orientation: " + currentOrientation.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent angular velocity: " + currentAngularVelocity.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent angular acceleration: " + currentAngularAcceleration.toStringForASingleReferenceFrame(currentFrame);
      return ret;
   }
}
