package us.ihmc.avatar.multiContact;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;

public class RobotTransformOptimizer
{
   private final RigidBodyReadOnly heaviestBodyA;
   private final RigidBodyReadOnly heaviestBodyB;
   private final RigidBodyReadOnly[] rigidBodiesA;
   private final RigidBodyReadOnly[] rigidBodiesB;
   private final List<RigidBodyPair> rigidBodyPairs = new ArrayList<>();

   private boolean initializeWithHeaviestBody = false;
   private int maxIterations = 100;
   private double convergenceThreshold = 1.0e-7;

   private final DMatrixRMaj perturbationVector = new DMatrixRMaj(6, 1);
   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(true);
   private final ErrorCalculator errorCalculator = new ErrorCalculator();
   private final RigidBodyTransform transformFromBToA = new RigidBodyTransform();

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

   public void configureBodiesToMatch(Predicate<RigidBodyReadOnly> bodySelector)
   {
      rigidBodyPairs.clear();

      for (int i = 0; i < rigidBodiesA.length; i++)
      {
         RigidBodyReadOnly bodyA = rigidBodiesA[i];
         RigidBodyReadOnly bodyB = rigidBodiesB[i];

         if (bodySelector.test(bodyA))
         {
            rigidBodyPairs.add(new RigidBodyPair(bodyA, bodyB));
         }
      }
   }

   public void compute()
   {
      DMatrixRMaj initialGuess = new DMatrixRMaj(6, 1);
      rigidBodyPairs.forEach(bodyPair -> bodyPair.initialize());

      if (initializeWithHeaviestBody)
      {
         RigidBodyTransform initialTransform = new RigidBodyTransform(heaviestBodyB.getBodyFixedFrame().getTransformToRoot());
         initialTransform.preMultiplyInvertThis(heaviestBodyA.getBodyFixedFrame().getTransformToRoot());
         System.out.println(initialTransform);
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
         rigidBodyPairs.forEach(bodyPair -> average.add(bodyPair.computeError()));
         average.scale(1.0 / rigidBodyPairs.size());
         average.get(initialGuess);
      }

      int inputParameterDimension = 6;
      int outputDimension = rigidBodyPairs.size();
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction,
                                                                                                errorCalculator,
                                                                                                inputParameterDimension,
                                                                                                outputDimension);
      optimizer.setInitialOptimalGuess(initialGuess);
      optimizer.setPerturbationVector(perturbationVector);
      optimizer.setCorrespondenceThreshold(Double.POSITIVE_INFINITY);
      optimizer.initialize();

      for (int i = 0; i < maxIterations; i++)
      {
         double quality = optimizer.iterate();
         System.out.println("Quality: " + quality);
         System.out.println("-----------------------------------------");

         if (quality <= convergenceThreshold)
            break;
      }

      transformFromBToA.set(inputFunction.apply(optimizer.getOptimalParameter()));
   }

   public RigidBodyTransform getTransformFromBToA()
   {
      return transformFromBToA;
   }

   private static class RigidBodyPair
   {
      private final RigidBodyReadOnly bodyA;
      private final RigidBodyReadOnly bodyB;

      private final Point3D pointInitialA = new Point3D();
      private final Point3D pointInitialB = new Point3D();
      private final Point3D pointCorrectedB = new Point3D();

      private final Vector3D error = new Vector3D();

      public RigidBodyPair(RigidBodyReadOnly bodyA, RigidBodyReadOnly bodyB)
      {
         this.bodyA = bodyA;
         this.bodyB = bodyB;
      }

      private void initialize()
      {
         pointInitialA.set(bodyA.getBodyFixedFrame().getTransformToRoot().getTranslation());
         pointInitialB.set(bodyB.getBodyFixedFrame().getTransformToRoot().getTranslation());
      }

      private Vector3D computeError()
      {
         error.sub(pointInitialA, pointInitialB);
         return error;
      }

      private Vector3D computeError(RigidBodyTransform transformForB)
      {
         transformForB.transform(pointInitialB, pointCorrectedB);
         error.sub(pointInitialA, pointCorrectedB);
         return error;
      }
   }

   private class ErrorCalculator implements OutputCalculator
   {
      @Override
      public DMatrixRMaj apply(DMatrixRMaj inputParameter)
      {
         RigidBodyTransform correction = inputFunction.apply(inputParameter);
         DMatrixRMaj errorSpace = new DMatrixRMaj(rigidBodyPairs.size(), 1);

         for (int i = 0; i < rigidBodyPairs.size(); i++)
         {
            RigidBodyPair bodyPair = rigidBodyPairs.get(i);
            errorSpace.set(i, bodyPair.computeError(correction).length());
         }
         return errorSpace;
      }
   }
}
