package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.euclid.referenceFrame.*;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class VelocityConstrainedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final YoMutableFramePoint3D initialPosition;
   private final YoMutableFrameVector3D initialVelocity;
   private final YoMutableFramePoint3D finalPosition;
   private final YoMutableFrameVector3D finalVelocity;
   private final YoFramePoint3D finalPositionForViz;

   private final YoMutableFrameQuaternion initialOrientation;
   private final YoMutableFrameVector3D initialAngularVelocity;
   private final YoMutableFrameQuaternion finalOrientation;
   private final YoMutableFrameVector3D finalAngularVelocity;
   private final YoFrameYawPitchRoll finalOrientationForViz;

   private final YoMutableFramePoint3D currentPosition;
   private final YoMutableFrameVector3D currentVelocity;
   private final YoMutableFrameVector3D currentAcceleration;

   private final YoMutableFrameQuaternion currentOrientation;
   private final YoMutableFrameVector3D currentAngularVelocity;
   private final YoMutableFrameVector3D currentAngularAcceleration;
   private final YoFrameYawPitchRoll currentOrientationForViz;

   private final FrameQuaternion tempCurrentOrientation;
   private final FrameVector3D tempCurrentAngularVelocity;
   private final FrameVector3D tempCurrentAngularAcceleration;
   private final AxisAngle tempAxisAngle;
   private final Vector3D tempVector;
   double vectorLength;

   private final double FDdt = 5e-6;
   private Quaternion quatFD1, quatFD3, quatFDDelta;
   private double deltaAngle, omegaFD;
   private FrameVector3D omegaVectorDF;

   private final FramePoint3D tempPosition;
   private final FrameQuaternion tempOrientation;
   private final FrameQuaternion copyOfInitialOrientation;
   private final FrameQuaternion copyOfFinalOrientation;
   private final FrameVector3D copyOfInitialAngularVelocity;
   private final FrameVector3D copyOfFinalAngularVelocity;

   private final YoPolynomial xPolynomial, yPolynomial, zPolynomial;
   private final YoPolynomial xRotPolynomial, yRotPolynomial, zRotPolynomial;

   private ReferenceFrame trajectoryFrame;
   private final ReferenceFrame interpolationFrame;
   private ReferenceFrame currentTrajectoryFrame;
   private ReferenceFrame finalFrame;
   private final YoFrameYawPitchRoll interpolationFrameForViz;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;

   // For Visualization
   private final boolean visualize;
   private final YoGraphicsList yoGraphicsList;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   private final List<ImmutablePair<FrameTuple3DReadOnly, FixedFrameTuple3DBasics>> visualizationUpdatables = new ArrayList<>();

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, referenceFrame, parentRegistry, false, null);
   }

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry, boolean visualize,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.trajectoryFrame = referenceFrame;

      quatFD1 = new Quaternion();
      quatFD3 = new Quaternion();
      quatFDDelta = new Quaternion();
      omegaVectorDF = new FrameVector3D();

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      initialPosition = new YoMutableFramePoint3D(namePrefix + "InitialPosition", "", registry, referenceFrame);
      initialVelocity = new YoMutableFrameVector3D(namePrefix + "InitialVelocity", "", registry, referenceFrame);
      finalPosition = new YoMutableFramePoint3D(namePrefix + "FinalPosition", "", registry, referenceFrame);
      finalVelocity = new YoMutableFrameVector3D(namePrefix + "FinalVelocity", "", registry, referenceFrame);
      finalPositionForViz = new YoFramePoint3D(namePrefix + "FinalPositionForViz", worldFrame, registry);

      initialOrientation = new YoMutableFrameQuaternion(namePrefix + "InitialOrientation", "", registry, referenceFrame);
      initialAngularVelocity = new YoMutableFrameVector3D(namePrefix + "InitialAngularVelocity", "", registry, referenceFrame);
      finalOrientation = new YoMutableFrameQuaternion(namePrefix + "FinalOrientation", "", registry, referenceFrame);
      finalAngularVelocity = new YoMutableFrameVector3D(namePrefix + "FinalAngularVelocity", "", registry, referenceFrame);
      finalOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "FinalOrientationForViz", worldFrame, registry);

      currentPosition = new YoMutableFramePoint3D(namePrefix + "CurrentPosition", "", registry, referenceFrame);
      currentVelocity = new YoMutableFrameVector3D(namePrefix + "CurrentVelocity", "", registry, referenceFrame);
      currentAcceleration = new YoMutableFrameVector3D(namePrefix + "CurrentAcceleration", "", registry, referenceFrame);

      currentOrientation = new YoMutableFrameQuaternion(namePrefix + "CurrentOrientation", "", registry, referenceFrame);
      currentAngularVelocity = new YoMutableFrameVector3D(namePrefix + "CurrentAngularVelocity", "", registry, referenceFrame);
      currentAngularAcceleration = new YoMutableFrameVector3D(namePrefix + "CurrentAngularAcceleration", "", registry, referenceFrame);
      currentOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "CurrentOrientationForViz", worldFrame, registry);

      tempCurrentOrientation = new FrameQuaternion();
      tempCurrentAngularVelocity = new FrameVector3D();
      tempCurrentAngularAcceleration = new FrameVector3D();
      tempAxisAngle = new AxisAngle();
      tempVector = new Vector3D();

      tempPosition = new FramePoint3D();
      tempOrientation = new FrameQuaternion(trajectoryFrame);
      copyOfInitialOrientation = new FrameQuaternion(trajectoryFrame);
      copyOfFinalOrientation = new FrameQuaternion(trajectoryFrame);
      copyOfInitialAngularVelocity = new FrameVector3D(trajectoryFrame);
      copyOfFinalAngularVelocity = new FrameVector3D(trajectoryFrame);

      currentTime = new YoDouble(namePrefix + "Time", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);

      xPolynomial = new YoPolynomial(namePrefix + "PolynomialX", 6, registry);
      yPolynomial = new YoPolynomial(namePrefix + "PolynomialY", 6, registry);
      zPolynomial = new YoPolynomial(namePrefix + "PolynomialZ", 6, registry);

      xRotPolynomial = new YoPolynomial(namePrefix + "PolynomialRoll", 6, registry);
      yRotPolynomial = new YoPolynomial(namePrefix + "PolynomialPitch", 6, registry);
      zRotPolynomial = new YoPolynomial(namePrefix + "PolynomialYaw", 6, registry);

      interpolationFrame = new ReferenceFrame("interPolationFrame", ReferenceFrame.getWorldFrame())
      {
         private final FrameQuaternion localFrameOrientation = new FrameQuaternion();
         private final RotationMatrix localRotation = new RotationMatrix();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localFrameOrientation.setIncludingFrame(initialOrientation);
            localFrameOrientation.changeFrame(getParent());
            localRotation.set(localFrameOrientation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      finalFrame = new ReferenceFrame("finalFrame", ReferenceFrame.getWorldFrame())
      {
         private final FrameQuaternion localFrameOrientation = new FrameQuaternion();
         private final RotationMatrix localRotation = new RotationMatrix();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localFrameOrientation.setIncludingFrame(initialOrientation);
            localFrameOrientation.changeFrame(getParent());
            localRotation.set(localFrameOrientation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      currentTrajectoryFrame = new ReferenceFrame("currentTrajectoryFrame", interpolationFrame)
      {
         private final FrameQuaternion localFrameOrientation = new FrameQuaternion();
         private final RotationMatrix localRotation = new RotationMatrix();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localFrameOrientation.setIncludingFrame(currentOrientation);
            localFrameOrientation.changeFrame(getParent());
            localRotation.set(localFrameOrientation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      interpolationFrameForViz = new YoFrameYawPitchRoll(namePrefix + "InterpolationFrameForViz", worldFrame, registry);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         YoFramePoint3D currentPositionInWorld = new YoFramePoint3D(namePrefix + "CurrentPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<>(currentPosition, currentPositionInWorld));
         YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPositionInWorld, 0.025, YoAppearance.Blue());

         YoFramePoint3D initialPositionInWorld = new YoFramePoint3D(namePrefix + "InitialPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<>(initialPosition, initialPositionInWorld));
         YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPositionInWorld, 0.02, YoAppearance.BlueViolet());

         YoFramePoint3D finalPositionInWorld = new YoFramePoint3D(namePrefix + "FinalPosition", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<>(finalPosition, finalPositionInWorld));
         YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPositionInWorld, 0.02, YoAppearance.Red());

         YoFrameVector3D currentLinearVelocityInWorld = new YoFrameVector3D(namePrefix + "CurrentLinearVelocity", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<>(currentVelocity, currentLinearVelocityInWorld));
         YoGraphicVector currentVelocityViz = new YoGraphicVector(namePrefix + "CurrentVelocity", currentPositionInWorld, currentLinearVelocityInWorld, 0.2,
                                                                  YoAppearance.Chartreuse());
         YoFrameVector3D currentAngularVelocityInWorld = new YoFrameVector3D(namePrefix + "CurrentAngularVelocity", "WorldViz", ReferenceFrame.getWorldFrame(), registry);
         visualizationUpdatables.add(new ImmutablePair<>(currentAngularVelocity, currentAngularVelocityInWorld));
         YoGraphicVector currentAngularVelocityViz = new YoGraphicVector(namePrefix + "CurrentAngularVelocity", currentPositionInWorld,
                                                                         currentAngularVelocityInWorld, 0.2, YoAppearance.Green());

         YoGraphicCoordinateSystem interpolationCoordinateSystemViz = new YoGraphicCoordinateSystem(namePrefix + "interpolationCoordinateSystem",
                                                                                                    initialPositionInWorld, interpolationFrameForViz, 0.3,
                                                                                                    YoAppearance.Black());
         YoGraphicCoordinateSystem currentPoseViz = new YoGraphicCoordinateSystem(namePrefix + "CurrentPose", currentPositionInWorld, currentOrientationForViz,
                                                                                  0.3);

         yoGraphicsList = new YoGraphicsList(namePrefix + "VelocityConstrainedTrajectory");
         //         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(currentVelocityViz);
         //         yoGraphicsList.add(initialPositionViz);
         //         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(currentPoseViz);
         yoGraphicsList.add(currentAngularVelocityViz);
         yoGraphicsList.add(interpolationCoordinateSystemViz);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowViz", registry);
         showViz.addListener(new YoVariableChangedListener()
         {
            @Override
            public void changed(YoVariable v)
            {
               boolean visible = showViz.getBooleanValue();
               currentVelocityViz.setVisible(visible);
               currentAngularVelocityViz.setVisible(visible);
               //               currentPositionViz.setVisible(visible);
               //               initialPositionViz.setVisible(visible);
               //               finalPositionViz.setVisible(visible);
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
      initialVelocity.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);
      finalVelocity.changeFrame(referenceFrame);
      currentPosition.changeFrame(referenceFrame);
      currentVelocity.changeFrame(referenceFrame);
      currentAcceleration.changeFrame(referenceFrame);

      initialOrientation.changeFrame(referenceFrame);
      initialAngularVelocity.changeFrame(referenceFrame);
      finalOrientation.changeFrame(referenceFrame);
      finalAngularVelocity.changeFrame(referenceFrame);
      currentOrientation.changeFrame(referenceFrame);
      currentAngularVelocity.changeFrame(referenceFrame);
      currentAngularAcceleration.changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      initialPosition.setToZero(referenceFrame);
      initialVelocity.setToZero(referenceFrame);
      finalPosition.setToZero(referenceFrame);
      finalVelocity.setToZero(referenceFrame);
      currentPosition.setToZero(referenceFrame);
      currentVelocity.setToZero(referenceFrame);
      currentAcceleration.setToZero(referenceFrame);

      initialOrientation.setToZero(referenceFrame);
      initialAngularVelocity.setToZero(referenceFrame);
      finalOrientation.setToZero(referenceFrame);
      finalAngularVelocity.setToZero(referenceFrame);
      currentOrientation.setToZero(referenceFrame);
      currentAngularVelocity.setToZero(referenceFrame);
      currentAngularAcceleration.setToZero(referenceFrame);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
   }

   public void setInitialPoseWithInitialVelocity(FramePose3D initialPose, FrameVector3D initialVelocity, FrameVector3D initialAngularVelocity)
   {
      initialPose.get(tempPosition, tempOrientation);
      this.initialPosition.set(tempPosition);
      this.initialVelocity.set(initialVelocity);

      this.initialOrientation.set(tempOrientation);
      this.initialAngularVelocity.set(initialAngularVelocity);
   }

   public void setInitialPoseWithoutInitialVelocity(FramePose3D initialPose)
   {
      initialPose.get(tempPosition, tempOrientation);
      this.initialPosition.set(tempPosition);
      this.initialVelocity.setToZero();
      ;

      this.initialOrientation.set(tempOrientation);
      this.initialAngularVelocity.setToZero();
   }

   public void setInitialPoseWithInitialVelocity(FramePoint3D initialPosition, FrameVector3D initialVelocity, FrameQuaternion initialOrientation,
         FrameVector3D initialAngularVelocity)
   {
      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
      this.initialOrientation.set(initialOrientation);
      this.initialAngularVelocity.set(initialAngularVelocity);
   }

   public void setInitialPoseWithoutInitialVelocity(FramePoint3D initialPosition, FrameQuaternion initialOrientation)
   {
      this.initialPosition.set(initialPosition);
      this.initialVelocity.setToZero();
      this.initialOrientation.set(initialOrientation);
      this.initialAngularVelocity.setToZero();
   }

   /**
    * TODO Get that working with final angular velocity.
    *
    */
   /*
    * public void setFinalPoseWithFinalVelocity(FramePose finalPose, FrameVector
    * finalVelocity, FrameVector finalAngularVelocity) {
    * finalPose.getPoseIncludingFrame(tempPosition, tempOrientation);
    * setFinalPoseWithFinalVelocity(tempPosition, tempOrientation,
    * finalVelocity, finalAngularVelocity); } public void
    * setFinalPoseWithFinalVelocity(FramePoint finalPosition, FrameOrientation
    * finalOrientation, FrameVector finalVelocity, FrameVector
    * finalAngularVelocity) { this.finalPosition.set(finalPosition);
    * this.finalOrientation.set(finalOrientation);
    * finalPositionForViz.setAndMatchFrame(finalPosition);
    * finalOrientationForViz.setAndMatchFrame(finalOrientation);
    * this.finalVelocity.set(finalVelocity);
    * this.finalAngularVelocity.set(finalAngularVelocity); }
    */
   public void setFinalPoseWithoutFinalVelocity(FramePose3D finalPose)
   {
      finalPose.get(tempPosition, tempOrientation);
      setFinalPoseWithoutFinalVelocity(tempPosition, tempOrientation);
   }

   public void setFinalPoseWithoutFinalVelocity(FramePoint3D finalPosition, FrameQuaternion finalOrientation)
   {
      this.finalPosition.set(finalPosition);
      this.finalOrientation.set(finalOrientation);

      finalPositionForViz.setMatchingFrame(finalPosition);
      finalOrientationForViz.setMatchingFrame(finalOrientation);

      this.finalVelocity.setToZero();
      this.finalAngularVelocity.setToZero();
   }

   @Override
   public void initialize()

   {
      interpolationFrame.update();
      finalFrame.update();
      copyOfInitialOrientation.setToZero(interpolationFrame);
      copyOfInitialOrientation.changeFrame(worldFrame);
      interpolationFrameForViz.set(copyOfInitialOrientation);

      trajectoryFrame = initialOrientation.getReferenceFrame();
      // Translational part
      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      xPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getX(), initialVelocity.getX(), 0.0, finalPosition.getX(),
            finalVelocity.getX(), 0.0);
      yPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getY(), initialVelocity.getY(), 0.0, finalPosition.getY(),
            finalVelocity.getY(), 0.0);
      zPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getZ(), initialVelocity.getZ(), 0.0, finalPosition.getZ(),
            finalVelocity.getZ(), 0.0);

      copyOfInitialOrientation.setIncludingFrame(initialOrientation);
      copyOfFinalOrientation.setIncludingFrame(finalOrientation);
      copyOfInitialAngularVelocity.setIncludingFrame(initialAngularVelocity);
      copyOfFinalAngularVelocity.setIncludingFrame(finalAngularVelocity);

      copyOfInitialOrientation.changeFrame(interpolationFrame);
      copyOfFinalOrientation.changeFrame(interpolationFrame);
      copyOfInitialAngularVelocity.changeFrame(interpolationFrame);
      copyOfFinalAngularVelocity.changeFrame(interpolationFrame);

      tempAxisAngle.set(copyOfFinalOrientation);

      /**
       * TODO Get that working with final angular velocity.
       * T. Meier:
       * Since the pose is rotating on the Path this approach is not correct: The derivative of the angles is not equal to the angular velocity (Orientation and angular velocity are not consistent!). To
       * avoid this the latter is calculated with finite differences of orientation quaternions. (See compute function). The FinalAngularVelocity as boundary condition is only working if it is 0.0!!
       * Interpolating over the angular velocity vector is not possible either since the pose is restrained as well.
       * A parametrisation of orientation which is consistent with a random angular velocity is needed. Searching for one.
       */
      xRotPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, copyOfInitialAngularVelocity.getX(), 0.0,
            tempAxisAngle.getX() * tempAxisAngle.getAngle(), copyOfFinalAngularVelocity.getX(), 0.0);
      yRotPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, copyOfInitialAngularVelocity.getY(), 0.0,
            tempAxisAngle.getY() * tempAxisAngle.getAngle(), copyOfFinalAngularVelocity.getY(), 0.0);
      zRotPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, copyOfInitialAngularVelocity.getZ(), 0.0,
            tempAxisAngle.getZ() * tempAxisAngle.getAngle(), copyOfFinalAngularVelocity.getZ(), 0.0);

      reset();

      if (visualize)
         visualizeTrajectory();
   }

   private void reset()
   {
      currentTime.set(0.0);
      currentPosition.set(initialPosition);
      currentVelocity.set(initialVelocity);
      currentAcceleration.setToZero();
      currentOrientation.set(initialOrientation);
      currentAngularVelocity.set(initialAngularVelocity);
      currentAngularAcceleration.setToZero();
   }

   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);
      currentTrajectoryFrame.update();
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());

      xPolynomial.compute(time);
      yPolynomial.compute(time);
      zPolynomial.compute(time);

      xRotPolynomial.compute(time);
      yRotPolynomial.compute(time);
      zRotPolynomial.compute(time);

      if (!isDone())
      {
         // Linear Part
         currentPosition.set(xPolynomial.getPosition(), yPolynomial.getPosition(), zPolynomial.getPosition());
         currentVelocity.set(xPolynomial.getVelocity(), yPolynomial.getVelocity(), zPolynomial.getVelocity());
         currentAcceleration.set(xPolynomial.getAcceleration(), yPolynomial.getAcceleration(), zPolynomial.getAcceleration());

         // Rotational Part: Transformation from interpolationFrame to trajectoryFrame
         tempVector.set(xRotPolynomial.getPosition(), yRotPolynomial.getPosition(), zRotPolynomial.getPosition());
         vectorLength = tempVector.length();
         if (vectorLength > 0.0)
         {
            tempVector.normalize();
            tempAxisAngle.set(tempVector.getX(), tempVector.getY(), tempVector.getZ(), vectorLength);
         }
         else
         {
            tempAxisAngle.set(1.0, 0.0, 0.0, 0.0);
         }
         tempCurrentOrientation.setIncludingFrame(interpolationFrame, tempAxisAngle);

         tempCurrentOrientation.changeFrame(trajectoryFrame);
         currentOrientation.set(tempCurrentOrientation);

         /**
          * Calculate velocity using finite differences to ensure consistency between orientation and angular velocity.
          */
         // t + FDdt
         xRotPolynomial.compute(time + FDdt);
         yRotPolynomial.compute(time + FDdt);
         zRotPolynomial.compute(time + FDdt);

         tempVector.set(xRotPolynomial.getPosition(), yRotPolynomial.getPosition(), zRotPolynomial.getPosition());
         vectorLength = tempVector.length();
         if (vectorLength > 0.0)
         {
            tempVector.normalize();
            tempAxisAngle.set(tempVector.getX(), tempVector.getY(), tempVector.getZ(), vectorLength);
         }
         else
         {
            tempAxisAngle.set(1.0, 0.0, 0.0, 0.0);
         }
         tempCurrentOrientation.setIncludingFrame(interpolationFrame, tempAxisAngle);
         tempCurrentOrientation.changeFrame(worldFrame);
         quatFD3.set(tempCurrentOrientation);

         // t - FDdt
         xRotPolynomial.compute(time - FDdt);
         yRotPolynomial.compute(time - FDdt);
         zRotPolynomial.compute(time - FDdt);

         tempVector.set(xRotPolynomial.getPosition(), yRotPolynomial.getPosition(), zRotPolynomial.getPosition());
         vectorLength = tempVector.length();
         if (vectorLength > 0.0)
         {
            tempVector.normalize();
         }
         tempAxisAngle.set(tempVector.getX(), tempVector.getY(), tempVector.getZ(), vectorLength);
         tempCurrentOrientation.setIncludingFrame(interpolationFrame, tempAxisAngle);
         tempCurrentOrientation.changeFrame(worldFrame);
         quatFD1.set(tempCurrentOrientation);

         // Finite Differences using quaternions
         quatFD1.inverse();
         quatFDDelta.multiply(quatFD3, quatFD1);
         quatFDDelta.normalize();

         deltaAngle = Math.acos(quatFDDelta.getS()) * 2.0;

         omegaFD = deltaAngle / (2.0 * FDdt);
         omegaVectorDF.setIncludingFrame(worldFrame, quatFDDelta.getX(), quatFDDelta.getY(), quatFDDelta.getZ());

         if (omegaVectorDF.length() > 0.0)
         {
            omegaVectorDF.normalize();
         }
         omegaVectorDF.scale(omegaFD);

         tempCurrentAngularVelocity.setIncludingFrame(omegaVectorDF);

         // Change back to trajectoryFrame
         tempCurrentAngularVelocity.changeFrame(trajectoryFrame);
         currentAngularVelocity.set(tempCurrentAngularVelocity);

         /*
          * tempVector.set(xRotPolynomial.getVelocity(),
          * yRotPolynomial.getVelocity(), zRotPolynomial.getVelocity());
          * tempCurrentAngularVelocity.setIncludingFrame(interpolationFrame,
          * tempVector); tempVector.set(xRotPolynomial.getAcceleration(),
          * yRotPolynomial.getAcceleration(), zRotPolynomial.getAcceleration());
          * tempCurrentAngularAcceleration.setIncludingFrame(interpolationFrame,
          * tempVector); tempCurrentOrientation.changeFrame(trajectoryFrame);
          * tempCurrentAngularVelocity.changeFrame(trajectoryFrame);
          * tempCurrentAngularAcceleration.changeFrame(trajectoryFrame);
          * currentOrientation.set(tempCurrentOrientation);
          * currentAngularVelocity.set(tempCurrentAngularVelocity);
          * currentAngularAcceleration.set(tempCurrentAngularAcceleration);
          */

      }
      else
      {
         currentPosition.set(finalPosition);
         currentVelocity.set(finalVelocity);
         currentAcceleration.set(0.0, 0.0, 0.0);

         currentOrientation.set(finalOrientation);
         currentAngularVelocity.set(finalAngularVelocity);
         currentAcceleration.set(0.0, 0.0, 0.0);
      }

      //      currentOrientationForViz.set(currentOrientation);
      currentOrientationForViz.set(currentTrajectoryFrame.getTransformToWorldFrame().getRotation());
      for (int i = 0; i < visualizationUpdatables.size(); i++)
      {
         visualizationUpdatables.get(i).getRight().setMatchingFrame(visualizationUpdatables.get(i).getLeft());
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
         currentOrientationForViz.set(currentOrientation);
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
