package us.ihmc.avatar.multiContact;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiPredicate;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * This class provides a generic solver to optimize the pose of a robot to match as closely as
 * possible another robot and this without changing the joint angles but the root joint pose.
 * <p>
 * Typical use case is when 2 robots are in similar configurations to estimate the best transform to
 * go from one to the other.
 * </p>
 */
public class RobotTransformOptimizer
{
   private final RigidBodyReadOnly heaviestBodyA;
   private final RigidBodyReadOnly heaviestBodyB;
   private final RigidBodyReadOnly[] rigidBodiesA;
   private final RigidBodyReadOnly[] rigidBodiesB;
   private final List<RigidBodyPairErrorCalculator> rigidBodyErrorCalculators = new ArrayList<>();
   private final Map<String, Pair<RigidBodyReadOnly, RigidBodyReadOnly>> nameAToBodyMap = new HashMap<>();
   private final Map<String, Pair<RigidBodyReadOnly, RigidBodyReadOnly>> nameBToBodyMap = new HashMap<>();

   private boolean initializeWithHeaviestBody = false;
   private int maxIterations = 500;
   private double convergenceThreshold = 1.0e-7;

   private final RigidBodyTransform transformFromBToA = new RigidBodyTransform();
   private final DMatrixRMaj perturbationVector = new DMatrixRMaj(6, 1);
   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction = input ->
   {
      transformFromBToA.getRotation().setYawPitchRoll(input.get(5), input.get(4), input.get(3));
      transformFromBToA.getTranslation().set(input.get(0), input.get(1), input.get(2));
      return transformFromBToA;
   };
   private final DMatrixRMaj errorSpace = new DMatrixRMaj(50, 1);
   private final OutputCalculator errorCalculator = inputParameter ->
   {
      errorSpace.reshape(rigidBodyErrorCalculators.size(), 1);
      RigidBodyTransform correction = inputFunction.apply(inputParameter);
      for (int i = 0; i < rigidBodyErrorCalculators.size(); i++)
         errorSpace.set(i, rigidBodyErrorCalculators.get(i).computeError(correction).length());
      return errorSpace;
   };

   private LevenbergMarquardtParameterOptimizer optimizer;

   /**
    * Creates a new solver given the root body of the two robots that are to be matched.
    * <p>
    * It is assumed that the configuration of each robot is updated outside this class.
    * </p>
    * 
    * @param rootBodyA the root body of the first robot.
    * @param rootBodyB the root body of the second robot.
    */
   public RobotTransformOptimizer(RigidBodyReadOnly rootBodyA, RigidBodyReadOnly rootBodyB)
   {
      rigidBodiesA = rootBodyA.subtreeArray();
      rigidBodiesB = rootBodyB.subtreeArray();

      RigidBodyReadOnly candidateA = rigidBodiesA[0];
      RigidBodyReadOnly candidateB = rigidBodiesB[0];

      for (int i = 0; i < rigidBodiesA.length; i++)
      {
         RigidBodyReadOnly bodyA = rigidBodiesA[i];
         RigidBodyReadOnly bodyB = rigidBodiesB[i];

         double currentMass = bodyA.isRootBody() ? 0 : bodyA.getInertia().getMass();
         double candidateMass = candidateA.isRootBody() ? 0 : candidateA.getInertia().getMass();

         if (currentMass > candidateMass)
         {
            candidateA = bodyA;
            candidateB = bodyB;
         }

         nameAToBodyMap.put(bodyA.getName(), new ImmutablePair<>(bodyA, bodyB));
         nameBToBodyMap.put(bodyB.getName(), new ImmutablePair<>(bodyA, bodyB));
      }

      heaviestBodyA = candidateA;
      heaviestBodyB = candidateB;

      perturbationVector.set(0, 0.000125);
      perturbationVector.set(1, 0.000125);
      perturbationVector.set(2, 0.000125);
      perturbationVector.set(3, 0.000025);
      perturbationVector.set(4, 0.000025);
      perturbationVector.set(5, 0.000025);
   }

   public void setInitializeWithHeaviestBody(boolean initializeWithHeaviestBody)
   {
      this.initializeWithHeaviestBody = initializeWithHeaviestBody;
   }

   public void clearErrorCalculators()
   {
      rigidBodyErrorCalculators.clear();
   }

   public void addDefaultRigidBodySpatialErrorCalculators(BiPredicate<RigidBodyReadOnly, RigidBodyReadOnly> bodySelector)
   {
      rigidBodyErrorCalculators.clear();

      for (int i = 0; i < rigidBodiesA.length; i++)
      {
         RigidBodyReadOnly bodyA = rigidBodiesA[i];
         RigidBodyReadOnly bodyB = rigidBodiesB[i];

         if (bodySelector.test(bodyA, bodyB))
         {
            rigidBodyErrorCalculators.add(new RigidBodyPairSpatialErrorCalculator(bodyA, bodyB));
         }
      }
   }

   public void addDefaultRigidBodyLinearErrorCalculators(BiPredicate<RigidBodyReadOnly, RigidBodyReadOnly> bodySelector)
   {
      rigidBodyErrorCalculators.clear();

      for (int i = 0; i < rigidBodiesA.length; i++)
      {
         RigidBodyReadOnly bodyA = rigidBodiesA[i];
         RigidBodyReadOnly bodyB = rigidBodiesB[i];

         if (bodySelector.test(bodyA, bodyB))
         {
            rigidBodyErrorCalculators.add(new RigidBodyPairLinearErrorCalculator(bodyA, bodyB));
         }
      }
   }

   public void addDefaultRigidBodyAngularErrorCalculators(BiPredicate<RigidBodyReadOnly, RigidBodyReadOnly> bodySelector)
   {
      rigidBodyErrorCalculators.clear();

      for (int i = 0; i < rigidBodiesA.length; i++)
      {
         RigidBodyReadOnly bodyA = rigidBodiesA[i];
         RigidBodyReadOnly bodyB = rigidBodiesB[i];

         if (bodySelector.test(bodyA, bodyB))
         {
            rigidBodyErrorCalculators.add(new RigidBodyPairAngularErrorCalculator(bodyA, bodyB));
         }
      }
   }

   public RigidBodyPairSpatialErrorCalculator addSpatialRigidBodyErrorCalculator(String bodyName)
   {
      Pair<RigidBodyReadOnly, RigidBodyReadOnly> bodyPair = nameAToBodyMap.get(bodyName);

      if (bodyPair == null)
         bodyPair = nameBToBodyMap.get(bodyName);

      if (bodyPair == null)
         return null;

      RigidBodyPairSpatialErrorCalculator calculator = new RigidBodyPairSpatialErrorCalculator(bodyPair.getLeft(), bodyPair.getRight());
      rigidBodyErrorCalculators.add(calculator);
      return calculator;
   }

   public RigidBodyPairLinearErrorCalculator addLinearRigidBodyErrorCalculator(String bodyName)
   {
      Pair<RigidBodyReadOnly, RigidBodyReadOnly> bodyPair = nameAToBodyMap.get(bodyName);

      if (bodyPair == null)
         bodyPair = nameBToBodyMap.get(bodyName);

      if (bodyPair == null)
         return null;

      RigidBodyPairLinearErrorCalculator calculator = new RigidBodyPairLinearErrorCalculator(bodyPair.getLeft(), bodyPair.getRight());
      rigidBodyErrorCalculators.add(calculator);
      return calculator;
   }

   public RigidBodyPairAngularErrorCalculator addAngularRigidBodyErrorCalculator(String bodyName)
   {
      Pair<RigidBodyReadOnly, RigidBodyReadOnly> bodyPair = nameAToBodyMap.get(bodyName);

      if (bodyPair == null)
         bodyPair = nameBToBodyMap.get(bodyName);

      if (bodyPair == null)
         return null;

      RigidBodyPairAngularErrorCalculator calculator = new RigidBodyPairAngularErrorCalculator(bodyPair.getLeft(), bodyPair.getRight());
      rigidBodyErrorCalculators.add(calculator);
      return calculator;
   }

   public void compute()
   {
      DMatrixRMaj initialGuess = new DMatrixRMaj(6, 1);
      rigidBodyErrorCalculators.forEach(bodyPair -> bodyPair.initialize());

      if (initializeWithHeaviestBody)
      {
         RigidBodyTransform initialTransform = new RigidBodyTransform(heaviestBodyB.getBodyFixedFrame().getTransformToRoot());
         initialTransform.preMultiplyInvertThis(heaviestBodyA.getBodyFixedFrame().getTransformToRoot());
         initialGuess.set(0, initialTransform.getTranslationX());
         initialGuess.set(1, initialTransform.getTranslationY());
         initialGuess.set(2, initialTransform.getTranslationZ());
         initialGuess.set(3, initialTransform.getRotation().getRoll());
         initialGuess.set(4, initialTransform.getRotation().getPitch());
         initialGuess.set(5, initialTransform.getRotation().getYaw());
      }
      else
      {
         Vector3D average = new Vector3D();
         rigidBodyErrorCalculators.forEach(bodyPair -> average.add(bodyPair.computeError().getLinearPart()));
         average.scale(1.0 / rigidBodyErrorCalculators.size());
         average.get(initialGuess);
      }

      int inputParameterDimension = 6;
      int outputDimension = rigidBodyErrorCalculators.size();
      optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction, errorCalculator, inputParameterDimension, outputDimension);
      optimizer.setInitialOptimalGuess(initialGuess);
      optimizer.setPerturbationVector(perturbationVector);
      optimizer.setCorrespondenceThreshold(Double.POSITIVE_INFINITY);
      optimizer.initialize();

      for (int i = 0; i < maxIterations; i++)
      {
         double quality = optimizer.iterate();

         if (quality <= convergenceThreshold)
            break;
      }

      transformFromBToA.set(inputFunction.apply(optimizer.getOptimalParameter()));
   }

   public RigidBodyTransform getTransformFromBToA()
   {
      return transformFromBToA;
   }

   public double getSolutionQuality()
   {
      if (optimizer != null)
         return optimizer.getQuality();
      else
         return Double.NaN;
   }

   public static abstract class RigidBodyPairErrorCalculator
   {
      protected final RigidBodyReadOnly bodyA;
      protected final RigidBodyReadOnly bodyB;

      protected final PoseReferenceFrame controlFrameA;
      protected final PoseReferenceFrame controlFrameB;

      protected RigidBodyPairErrorCalculator(RigidBodyReadOnly bodyA, RigidBodyReadOnly bodyB)
      {
         this.bodyA = bodyA;
         this.bodyB = bodyB;

         controlFrameA = new PoseReferenceFrame(bodyA.getName() + "ControlFrame", bodyA.getBodyFixedFrame());
         controlFrameB = new PoseReferenceFrame(bodyB.getName() + "ControlFrame", bodyB.getBodyFixedFrame());
      }

      protected abstract void initialize();

      protected SpatialVector computeError()
      {
         return computeError(null);
      }

      protected abstract SpatialVector computeError(RigidBodyTransform transformForB);

      public abstract SpatialVectorReadOnly getError();

      public RigidBodyReadOnly getBodyA()
      {
         return bodyA;
      }

      public RigidBodyReadOnly getBodyB()
      {
         return bodyB;
      }

      public void setControlFrameOffset(RigidBodyTransform controlFrameOffset)
      {
         controlFrameA.setTransformAndUpdate(controlFrameOffset);
         controlFrameB.setTransformAndUpdate(controlFrameOffset);
      }

      public ReferenceFrame getControlFrameA()
      {
         return controlFrameA;
      }

      public ReferenceFrame getControlFrameB()
      {
         return controlFrameB;
      }
   }

   public static class RigidBodyPairSpatialErrorCalculator extends RigidBodyPairErrorCalculator
   {
      private final RigidBodyTransform poseInitialA = new RigidBodyTransform();
      private final RigidBodyTransform poseInitialB = new RigidBodyTransform();

      private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
      private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      private final RigidBodyTransform errorTransform = new RigidBodyTransform();
      private final SpatialVector error = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector subSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector weightedSubSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());

      private final FrameVector3D tempError3D = new FrameVector3D(ReferenceFrame.getWorldFrame());

      protected RigidBodyPairSpatialErrorCalculator(RigidBodyReadOnly bodyA, RigidBodyReadOnly bodyB)
      {
         super(bodyA, bodyB);

         // By default the weights are all NaNs
         weightMatrix.setAngularWeights(1.0, 1.0, 1.0);
         weightMatrix.setLinearWeights(1.0, 1.0, 1.0);
      }

      @Override
      protected void initialize()
      {
         poseInitialA.set(controlFrameA.getTransformToRoot());
         poseInitialB.set(controlFrameB.getTransformToRoot());
      }

      @Override
      protected SpatialVector computeError(RigidBodyTransform transformForB)
      {
         errorTransform.set(poseInitialA);
         errorTransform.multiplyInvertOther(poseInitialB);
         if (transformForB != null)
            errorTransform.multiplyInvertOther(transformForB);

         errorTransform.getRotation().getRotationVector(error.getAngularPart());
         error.getLinearPart().set(errorTransform.getTranslation());

         tempError3D.setIncludingFrame(error.getAngularPart());
         selectionMatrix.applyAngularSelection(tempError3D);
         subSpaceError.getAngularPart().set(tempError3D);
         tempError3D.setIncludingFrame(error.getLinearPart());
         selectionMatrix.applyLinearSelection(tempError3D);
         subSpaceError.getLinearPart().set(tempError3D);

         tempError3D.setIncludingFrame(subSpaceError.getAngularPart());
         weightMatrix.applyAngularWeight(tempError3D);
         weightedSubSpaceError.getAngularPart().set(tempError3D);
         tempError3D.setIncludingFrame(subSpaceError.getLinearPart());
         weightMatrix.applyLinearWeight(tempError3D);
         weightedSubSpaceError.getLinearPart().set(tempError3D);

         return weightedSubSpaceError;
      }

      @Override
      public SpatialVectorReadOnly getError()
      {
         return weightedSubSpaceError;
      }

      public SelectionMatrix6D getSelectionMatrix()
      {
         return selectionMatrix;
      }

      public WeightMatrix6D getWeightMatrix()
      {
         return weightMatrix;
      }
   }

   public static class RigidBodyPairLinearErrorCalculator extends RigidBodyPairErrorCalculator
   {
      private final Point3D pointInitialA = new Point3D();
      private final Point3D pointInitialB = new Point3D();
      private final Point3D pointCorrectedB = new Point3D();

      private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
      private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

      private final SpatialVector error = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector subSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector weightedSubSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());

      private final FrameVector3D tempError3D = new FrameVector3D(ReferenceFrame.getWorldFrame());

      protected RigidBodyPairLinearErrorCalculator(RigidBodyReadOnly bodyA, RigidBodyReadOnly bodyB)
      {
         super(bodyA, bodyB);

         // By default the weights are all NaNs
         weightMatrix.setWeights(1.0, 1.0, 1.0);
      }

      @Override
      protected void initialize()
      {
         pointInitialA.set(controlFrameA.getTransformToRoot().getTranslation());
         pointInitialB.set(controlFrameB.getTransformToRoot().getTranslation());
      }

      @Override
      protected SpatialVector computeError(RigidBodyTransform transformForB)
      {
         if (transformForB != null)
         {
            transformForB.transform(pointInitialB, pointCorrectedB);
            error.getLinearPart().sub(pointInitialA, pointCorrectedB);
         }
         else
         {
            error.getLinearPart().sub(pointInitialA, pointInitialB);
         }

         tempError3D.setIncludingFrame(error.getLinearPart());
         selectionMatrix.applySelection(tempError3D);
         subSpaceError.getLinearPart().set(tempError3D);

         tempError3D.setIncludingFrame(subSpaceError.getLinearPart());
         weightMatrix.applyWeight(tempError3D);
         weightedSubSpaceError.getLinearPart().set(tempError3D);

         return weightedSubSpaceError;
      }

      @Override
      public SpatialVectorReadOnly getError()
      {
         return weightedSubSpaceError;
      }

      public SelectionMatrix3D getSelectionMatrix()
      {
         return selectionMatrix;
      }

      public WeightMatrix3D getWeightMatrix()
      {
         return weightMatrix;
      }
   }

   public static class RigidBodyPairAngularErrorCalculator extends RigidBodyPairErrorCalculator
   {
      private final Quaternion orientationInitialA = new Quaternion();
      private final Quaternion orientationInitialB = new Quaternion();

      private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
      private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

      private final Quaternion orientationError = new Quaternion();
      private final SpatialVector error = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector subSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());
      private final SpatialVector weightedSubSpaceError = new SpatialVector(ReferenceFrame.getWorldFrame());

      private final FrameVector3D tempError3D = new FrameVector3D(ReferenceFrame.getWorldFrame());

      protected RigidBodyPairAngularErrorCalculator(RigidBodyReadOnly bodyA, RigidBodyReadOnly bodyB)
      {
         super(bodyA, bodyB);

         // By default the weights are all NaNs
         weightMatrix.setWeights(1.0, 1.0, 1.0);
      }

      @Override
      protected void initialize()
      {
         orientationInitialA.set(controlFrameA.getTransformToRoot().getRotation());
         orientationInitialB.set(controlFrameB.getTransformToRoot().getRotation());
      }

      @Override
      protected SpatialVector computeError(RigidBodyTransform transformForB)
      {
         orientationError.set(orientationInitialA);
         orientationError.multiplyConjugateOther(orientationInitialB);
         if (transformForB != null)
            orientationError.appendInvertOther(transformForB.getRotation());

         orientationError.getRotationVector(error.getAngularPart());

         tempError3D.setIncludingFrame(error.getAngularPart());
         selectionMatrix.applySelection(tempError3D);
         subSpaceError.getAngularPart().set(tempError3D);

         tempError3D.setIncludingFrame(subSpaceError.getAngularPart());
         weightMatrix.applyWeight(tempError3D);
         weightedSubSpaceError.getAngularPart().set(tempError3D);

         return weightedSubSpaceError;
      }

      @Override
      public SpatialVectorReadOnly getError()
      {
         return weightedSubSpaceError;
      }

      public SelectionMatrix3D getSelectionMatrix()
      {
         return selectionMatrix;
      }

      public WeightMatrix3D getWeightMatrix()
      {
         return weightMatrix;
      }
   }
}
