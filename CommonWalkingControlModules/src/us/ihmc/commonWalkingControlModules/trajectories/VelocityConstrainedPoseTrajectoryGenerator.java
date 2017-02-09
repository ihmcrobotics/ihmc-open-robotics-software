package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class VelocityConstrainedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final boolean allowMultipleFrames;
   private final YoVariableRegistry registry;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFrameVectorInMultipleFrames initialVelocity;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames finalVelocity;
   private final YoFramePoint finalPositionForViz;

   private final YoFrameQuaternionInMultipleFrames initialOrientation;
   private final YoFrameVectorInMultipleFrames initialAngularVelocity;
   private final YoFrameQuaternionInMultipleFrames finalOrientation;
   private final YoFrameVectorInMultipleFrames finalAngularVelocity;
   private final YoFrameOrientation finalOrientationForViz;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoFrameQuaternionInMultipleFrames currentOrientation;
   private final YoFrameVectorInMultipleFrames currentAngularVelocity;
   private final YoFrameVectorInMultipleFrames currentAngularAcceleration;
   private final YoFrameOrientation currentOrientationForViz;

   private final FrameOrientation tempCurrentOrientation;
   private final FrameVector tempCurrentAngularVelocity;
   private final FrameVector tempCurrentAngularAcceleration;
   private final AxisAngle4d tempAxisAngle;
   private final Vector3d tempVector;
   double vectorLength;

   private final double FDdt = 5e-6;
   private Quat4d quatFD1, quatFD3, quatFDDelta;
   private double deltaAngle, omegaFD;
   private FrameVector omegaVectorDF;

   private final FramePoint tempPosition;
   private final FrameOrientation tempOrientation;
   private final FrameOrientation copyOfInitialOrientation;
   private final FrameOrientation copyOfFinalOrientation;
   private final FrameVector copyOfInitialAngularVelocity;
   private final FrameVector copyOfFinalAngularVelocity;

   private final YoPolynomial xPolynomial, yPolynomial, zPolynomial;
   private final YoPolynomial xRotPolynomial, yRotPolynomial, zRotPolynomial;

   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;

   private ReferenceFrame trajectoryFrame;
   private final ReferenceFrame interpolationFrame;
   private ReferenceFrame currentTrajectoryFrame;
   private ReferenceFrame finalFrame;
   private final YoFrameOrientation interpolationFrameForViz;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;

   // For Visualization
   private final boolean visualize;
   private final YoGraphicsList yoGraphicsList;
   private final BagOfBalls bagOfBalls;
   private final FramePoint ballPosition = new FramePoint();
   private final int numberOfBalls = 50;

   /** Use a BooleanYoVariable to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final BooleanYoVariable showViz;

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

      quatFD1 = new Quat4d();
      quatFD3 = new Quat4d();
      quatFDDelta = new Quat4d();
      omegaVectorDF = new FrameVector();

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame);
      initialVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "InitialVelocity", registry, referenceFrame);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame);
      finalVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "FinalVelocity", registry, referenceFrame);
      finalPositionForViz = new YoFramePoint(namePrefix + "FinalPositionForViz", worldFrame, registry);

      initialOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "InitialOrientation", registry, referenceFrame);
      initialAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "InitialAngularVelocity", registry, referenceFrame);
      finalOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "FinalOrientation", registry, referenceFrame);
      finalAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "FinalAngularVelocity", registry, referenceFrame);
      finalOrientationForViz = new YoFrameOrientation(namePrefix + "FinalOrientationForViz", worldFrame, registry);

      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame);

      currentOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + "CurrentOrientation", registry, referenceFrame);
      currentAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularVelocity", registry, referenceFrame);
      currentAngularAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAngularAcceleration", registry, referenceFrame);
      currentOrientationForViz = new YoFrameOrientation(namePrefix + "CurrentOrientationForViz", worldFrame, registry);

      tempCurrentOrientation = new FrameOrientation();
      tempCurrentAngularVelocity = new FrameVector();
      tempCurrentAngularAcceleration = new FrameVector();
      tempAxisAngle = new AxisAngle4d();
      tempVector = new Vector3d();

      tempPosition = new FramePoint();
      tempOrientation = new FrameOrientation(trajectoryFrame);
      copyOfInitialOrientation = new FrameOrientation(trajectoryFrame);
      copyOfFinalOrientation = new FrameOrientation(trajectoryFrame);
      copyOfInitialAngularVelocity = new FrameVector(trajectoryFrame);
      copyOfFinalAngularVelocity = new FrameVector(trajectoryFrame);

      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);

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
         private final FrameOrientation localFrameOrientation = new FrameOrientation();
         private final Matrix3d localRotation = new Matrix3d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialOrientation.getFrameOrientationIncludingFrame(localFrameOrientation);
            localFrameOrientation.changeFrame(parentFrame);
            localFrameOrientation.getMatrix3d(localRotation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      finalFrame = new ReferenceFrame("finalFrame", ReferenceFrame.getWorldFrame())
      {
         private final FrameOrientation localFrameOrientation = new FrameOrientation();
         private final Matrix3d localRotation = new Matrix3d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialOrientation.getFrameOrientationIncludingFrame(localFrameOrientation);
            localFrameOrientation.changeFrame(parentFrame);
            localFrameOrientation.getMatrix3d(localRotation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      currentTrajectoryFrame = new ReferenceFrame("currentTrajectoryFrame", interpolationFrame)
      {
         private final FrameOrientation localFrameOrientation = new FrameOrientation();
         private final Matrix3d localRotation = new Matrix3d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            currentOrientation.getFrameOrientationIncludingFrame(localFrameOrientation);
            localFrameOrientation.changeFrame(parentFrame);
            localFrameOrientation.getMatrix3d(localRotation);
            transformToParent.setRotationAndZeroTranslation(localRotation);
         }
      };

      interpolationFrameForViz = new YoFrameOrientation(namePrefix + "InterpolationFrameForViz", worldFrame, registry);

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

         showViz = new BooleanYoVariable(namePrefix + "ShowViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            public void variableChanged(YoVariable<?> v)
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

   public void setInitialPoseWithInitialVelocity(FramePose initialPose, FrameVector initialVelocity, FrameVector initialAngularVelocity)
   {
      initialPose.getPoseIncludingFrame(tempPosition, tempOrientation);
      this.initialPosition.set(tempPosition);
      this.initialVelocity.set(initialVelocity);

      this.initialOrientation.set(tempOrientation);
      this.initialAngularVelocity.set(initialAngularVelocity);
   }

   public void setInitialPoseWithoutInitialVelocity(FramePose initialPose)
   {
      initialPose.getPoseIncludingFrame(tempPosition, tempOrientation);
      this.initialPosition.set(tempPosition);
      this.initialVelocity.setToZero();
      ;

      this.initialOrientation.set(tempOrientation);
      this.initialAngularVelocity.setToZero();
   }

   public void setInitialPoseWithInitialVelocity(FramePoint initialPosition, FrameVector initialVelocity, FrameOrientation initialOrientation,
         FrameVector initialAngularVelocity)
   {
      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
      this.initialOrientation.set(initialOrientation);
      this.initialAngularVelocity.set(initialAngularVelocity);
   }

   public void setInitialPoseWithoutInitialVelocity(FramePoint initialPosition, FrameOrientation initialOrientation)
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
   public void setFinalPoseWithoutFinalVelocity(FramePose finalPose)
   {
      finalPose.getPoseIncludingFrame(tempPosition, tempOrientation);
      setFinalPoseWithoutFinalVelocity(tempPosition, tempOrientation);
   }

   public void setFinalPoseWithoutFinalVelocity(FramePoint finalPosition, FrameOrientation finalOrientation)
   {
      this.finalPosition.set(finalPosition);
      this.finalOrientation.set(finalOrientation);

      finalPositionForViz.setAndMatchFrame(finalPosition);
      finalOrientationForViz.setAndMatchFrame(finalOrientation);

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
      MathTools.checkIfInRange(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      xPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getX(), initialVelocity.getX(), 0.0, finalPosition.getX(),
            finalVelocity.getX(), 0.0);
      yPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getY(), initialVelocity.getY(), 0.0, finalPosition.getY(),
            finalVelocity.getY(), 0.0);
      zPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition.getZ(), initialVelocity.getZ(), 0.0, finalPosition.getZ(),
            finalVelocity.getZ(), 0.0);

      initialOrientation.getFrameOrientationIncludingFrame(copyOfInitialOrientation);
      finalOrientation.getFrameOrientationIncludingFrame(copyOfFinalOrientation);
      initialAngularVelocity.getFrameTupleIncludingFrame(copyOfInitialAngularVelocity);
      finalAngularVelocity.getFrameTupleIncludingFrame(copyOfFinalAngularVelocity);

      copyOfInitialOrientation.changeFrame(interpolationFrame);
      copyOfFinalOrientation.changeFrame(interpolationFrame);
      copyOfInitialAngularVelocity.changeFrame(interpolationFrame);
      copyOfFinalAngularVelocity.changeFrame(interpolationFrame);

      copyOfFinalOrientation.getAxisAngle(tempAxisAngle);

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
      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());

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
         tempCurrentOrientation.getQuaternion(quatFD3);

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
         tempCurrentOrientation.getQuaternion(quatFD1);

         // Finite Differences using quaternions
         quatFD1.inverse();
         quatFDDelta.mul(quatFD3, quatFD1);
         quatFDDelta.normalize();

         deltaAngle = Math.acos(quatFDDelta.getW()) * 2.0;

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
         currentPosition.getFrameTupleIncludingFrame(ballPosition);
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

   public void getPosition(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   public void getAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(angularAccelerationToPack);
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   private final Quat4d temp = new Quat4d();

   public void getPose(FramePose framePoseToPack)
   {
      framePoseToPack.changeFrame(currentPosition.getReferenceFrame());
      framePoseToPack.setPosition(currentPosition.getFrameTuple());

      currentOrientation.get(temp);
      framePoseToPack.setOrientation(temp);
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
