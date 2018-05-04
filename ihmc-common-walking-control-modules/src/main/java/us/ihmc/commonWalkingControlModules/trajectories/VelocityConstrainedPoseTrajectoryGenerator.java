package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;

public class VelocityConstrainedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final boolean allowMultipleFrames;
   private final YoVariableRegistry registry;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFrameVectorInMultipleFrames initialVelocity;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames finalVelocity;
   private final YoFramePoint3D finalPositionForViz;

   private final YoFrameQuaternionInMultipleFrames initialOrientation;
   private final YoFrameVectorInMultipleFrames initialAngularVelocity;
   private final YoFrameQuaternionInMultipleFrames finalOrientation;
   private final YoFrameVectorInMultipleFrames finalAngularVelocity;
   private final YoFrameYawPitchRoll finalOrientationForViz;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoFrameQuaternionInMultipleFrames currentOrientation;
   private final YoFrameVectorInMultipleFrames currentAngularVelocity;
   private final YoFrameVectorInMultipleFrames currentAngularAcceleration;
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

   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;

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

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public VelocityConstrainedPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry, boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.allowMultipleFrames = allowMultipleFrames;
      this.trajectoryFrame = referenceFrame;

      quatFD1 = new Quaternion();
      quatFD3 = new Quaternion();
      quatFDDelta = new Quaternion();
      omegaVectorDF = new FrameVector3D();

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame);
      initialVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "InitialVelocity", registry, referenceFrame);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame);
      finalVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "FinalVelocity", registry, referenceFrame);
      finalPositionForViz = new YoFramePoint3D(namePrefix + "FinalPositionForViz", worldFrame, registry);

      initialOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "InitialOrientation", registry, referenceFrame);
      initialAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "InitialAngularVelocity", registry, referenceFrame);
      finalOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "FinalOrientation", registry, referenceFrame);
      finalAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "FinalAngularVelocity", registry, referenceFrame);
      finalOrientationForViz = new YoFrameYawPitchRoll(namePrefix + "FinalOrientationForViz", worldFrame, registry);

      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame);

      currentOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "CurrentOrientation", registry, referenceFrame);
      currentAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularVelocity", registry, referenceFrame);
      currentAngularAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularAcceleration", registry, referenceFrame);
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

      multipleFramesHolders = new ArrayList<YoMultipleFramesHolder>();
      registerMultipleFramesHolders(initialPosition, initialVelocity, finalPosition, finalVelocity, currentPosition, currentVelocity, currentAcceleration);
      registerMultipleFramesHolders(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity, currentOrientation,
            currentAngularVelocity, currentAngularAcceleration);

      interpolationFrame = new ReferenceFrame("interPolationFrame", ReferenceFrame.getWorldFrame())
      {
         private final FrameQuaternion localFrameOrientation = new FrameQuaternion();
         private final RotationMatrix localRotation = new RotationMatrix();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            localFrameOrientation.setIncludingFrame(initialOrientation);
            localFrameOrientation.changeFrame(parentFrame);
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
            localFrameOrientation.changeFrame(parentFrame);
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
            localFrameOrientation.changeFrame(parentFrame);
            localRotation.set(localFrameOrientation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      interpolationFrameForViz = new YoFrameYawPitchRoll(namePrefix + "InterpolationFrameForViz", worldFrame, registry);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicVector currentVelocityViz = new YoGraphicVector(namePrefix + "CurrentVelocity",
               currentPosition.buildUpdatedYoFramePointForVisualizationOnly(), currentVelocity.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.Chartreuse());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
         final YoGraphicVector currentAngularVelocityViz = new YoGraphicVector(namePrefix + "CurrentAngularVelocity",
               currentPosition.buildUpdatedYoFramePointForVisualizationOnly(), currentAngularVelocity.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.Green());

         final YoGraphicCoordinateSystem interpolationCoordinateSystemViz = new YoGraphicCoordinateSystem(namePrefix + "interpolationCoordinateSystem",
               initialPosition.buildUpdatedYoFramePointForVisualizationOnly(), interpolationFrameForViz, 0.3, YoAppearance.Black());

         final YoGraphicCoordinateSystem currentPoseViz = new YoGraphicCoordinateSystem(namePrefix + "CurrentPose",
               currentPosition.buildUpdatedYoFramePointForVisualizationOnly(), currentOrientationForViz, 0.3);

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
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            public void notifyOfVariableChange(YoVariable<?> v)
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
      currentOrientationForViz.set(currentTrajectoryFrame.getTransformToWorldFrame());

   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = (double) i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         ballPosition.setIncludingFrame(currentPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         currentOrientationForViz.set(currentOrientation);
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
