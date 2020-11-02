package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.EstimatorContactPoint;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ForceEstimatorDynamicMatrixUpdater;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.JointspaceExternalContactEstimator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

/**
 * Implementation of the particle-filter based external contact estimator presented here:
 * http://groups.csail.mit.edu/robotics-center/public_papers/Manuelli16.pdf
 */
public class ContactParticleFilter implements RobotController
{
   private static final int numberOfParticles = 50;
   private static final int estimationVariables = 3;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final Random random = new Random(34098);

   private RigidBodyBasics rigidBody = null;

   private final YoDouble coefficientOfFriction = new YoDouble("coefficientOfFriction", registry);
   private final YoDouble particleMotionStandardDeviation = new YoDouble("particleMotionStandardDeviation", registry);

   private final JointBasics[] joints;
   private final int dofs;
   private final JointspaceExternalContactEstimator jointspaceExternalContactEstimator;
   private final DMatrixRMaj systemJacobian;
   private final ContactPointEvaluator contactPointEvaluator = new ContactPointEvaluator();

   private final EstimatorContactPoint[] contactPoints = new EstimatorContactPoint[numberOfParticles];
   private final ContactPointProjector[] contactPointProjectors = new ContactPointProjector[numberOfParticles];
   private final YoDouble[] contactPointProbabilities = new YoDouble[numberOfParticles];
   private final FramePoint3D[] sampledContactPositions = new FramePoint3D[numberOfParticles];
   private final YoFramePoint3D[] contactPointPositions = new YoFramePoint3D[numberOfParticles];
   private final YoFrameVector3D[] scaledSurfaceNormal = new YoFrameVector3D[numberOfParticles];

   private final double[] histogramValues = new double[numberOfParticles];
   private final int[] contactPointIndicesToSample = new int[numberOfParticles];

   private final FramePoint3D pointToProject = new FramePoint3D();

   // debugging variables
   private final YoFramePoint3D debugContactPoint = new YoFramePoint3D("debugContactPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D debugSurfaceNormal = new YoFrameVector3D("debugSurfaceNormal", ReferenceFrame.getWorldFrame(), registry);

   public ContactParticleFilter(JointBasics[] joints,
                                double dt,
                                ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater,
                                List<Collidable> collidables,
                                YoGraphicsListRegistry graphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.joints = joints;
      this.jointspaceExternalContactEstimator = new JointspaceExternalContactEstimator(joints, dt, dynamicMatrixUpdater, registry);
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      this.systemJacobian = new DMatrixRMaj(estimationVariables, dofs);

      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPointProbabilities[i] = new YoDouble("cpProb_" + i, registry);
         contactPointPositions[i] = new YoFramePoint3D("cpPosition_" + i, ReferenceFrame.getWorldFrame(), registry);
         scaledSurfaceNormal[i] = new YoFrameVector3D("cpNorm_" + i, ReferenceFrame.getWorldFrame(), registry);
         contactPointProjectors[i] = new ContactPointProjector(collidables);
         sampledContactPositions[i] = new FramePoint3D();

         if (graphicsListRegistry != null)
         {
            YoGraphicPosition contactPointGraphic = new YoGraphicPosition("cpPositionViz_" + i, contactPointPositions[i], 0.007, YoAppearance.Blue());
            YoGraphicVector contactVectorGraphic = new YoGraphicVector("cpNormViz_" + i, contactPointPositions[i], scaledSurfaceNormal[i], 1.0, YoAppearance.Blue());
            graphicsListRegistry.registerYoGraphic(name, contactPointGraphic);
            graphicsListRegistry.registerYoGraphic(name, contactVectorGraphic);
         }
      }

      if (graphicsListRegistry != null)
      {
         YoGraphicPosition debugContactPointGraphic = new YoGraphicPosition("cpPositionVizDebug", debugContactPoint, 0.025, YoAppearance.Red());
         YoGraphicVector debugContactVectorGraphic = new YoGraphicVector("cpNormVizDebug", debugContactPoint, debugSurfaceNormal, 1.0, YoAppearance.Red());
         graphicsListRegistry.registerYoGraphic(name, debugContactPointGraphic);
         graphicsListRegistry.registerYoGraphic(name, debugContactVectorGraphic);
      }

      coefficientOfFriction.set(0.5);
      particleMotionStandardDeviation.set(0.07);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void setLinkToEstimate(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;

      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPoints[i] = new EstimatorContactPoint(joints, rigidBody, true);
         contactPointProjectors[i].initialize(rigidBody);
      }
   }

   @Override
   public void initialize()
   {
      if (rigidBody == null)
      {
         throw new RuntimeException("Must set estimation link before initializing");
      }

      jointspaceExternalContactEstimator.initialize();
      contactPointEvaluator.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());

      computeInitialProjection();
   }

   public void computeInitialProjection()
   {
      double initialProjectionRadius = 1.0;
      for (int i = 0; i < numberOfParticles; i++)
      {
         Vector3D randomVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, initialProjectionRadius);
         pointToProject.setIncludingFrame(rigidBody.getBodyFixedFrame(), randomVector);
         pointToProject.changeFrame(ReferenceFrame.getWorldFrame());

         contactPointProjectors[i].computeProjection(pointToProject);
         contactPoints[i].setContactPointOffset(pointToProject);
      }
   }

   @Override
   public void doControl()
   {
      jointspaceExternalContactEstimator.doControl();
      DMatrixRMaj observedExternalJointTorque = jointspaceExternalContactEstimator.getObservedExternalJointTorque();

      double totalWeight = 0.0;

      // Evaluate likelihood of each point
      for (int i = 0; i < numberOfParticles; i++)
      {
         EstimatorContactPoint contactPoint = contactPoints[i];
         DMatrixRMaj contactPointJacobian = contactPoint.computeContactJacobian();

         for (int j = 0; j < contactPointJacobian.getNumCols(); j++)
         {
            int column = contactPoint.getSystemJacobianIndex(j);
            MatrixTools.setMatrixBlock(systemJacobian, 0, column, contactPointJacobian, 0, j, estimationVariables, 1, 1.0);
         }

         double likelihoodCost = contactPointEvaluator.computeMaximumLikelihoodForce(observedExternalJointTorque,
                                                                                     systemJacobian,
                                                                                     contactPointProjectors[i].getSurfaceFrame());
         double likelihoodWeight = Math.exp(-0.5 * likelihoodCost);
         contactPointProbabilities[i].set(likelihoodWeight);
         totalWeight += likelihoodWeight;
      }

      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPointProbabilities[i].mul(1.0 / totalWeight);

         double previousHistogramValue = i == 0 ? 0.0 : histogramValues[i - 1];
         histogramValues[i] = previousHistogramValue + contactPointProbabilities[i].getDoubleValue();
      }

      // Select points to resample
      for (int i = 0; i < numberOfParticles; i++)
      {
         // avoid edge case of sampling 1.0 and missing the end of the histogram
         double randomDouble = random.nextDouble() * (1.0 - 1e-4);

         for (int j = 0; j < histogramValues.length; j++)
         {
            if (histogramValues[j] > randomDouble)
            {
               contactPointIndicesToSample[i] = j;
               break;
            }
         }
      }

      // Perform resample
      for (int i = 0; i < numberOfParticles; i++)
      {
         int resampleIndex = contactPointIndicesToSample[i];
         ContactPointProjector pointProjector = contactPointProjectors[resampleIndex];

         boolean foundPointOutsideMesh = false;
         while (!foundPointOutsideMesh)
         {
            Vector3D offset = generateRandomOffset();
            FramePoint3D sampledContactPosition = sampledContactPositions[i];
            sampledContactPosition.setIncludingFrame(pointProjector.getSurfaceFrame(), offset);
            if (!contactPointProjectors[i].isPointInside(sampledContactPosition))
            {
               foundPointOutsideMesh = true;
            }
         }
      }

      // Project to surface
      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPointProjectors[i].computeProjection(sampledContactPositions[i]);
         contactPoints[i].setContactPointOffset(sampledContactPositions[i]);

         contactPointProjectors[i].getSurfacePoint().changeFrame(ReferenceFrame.getWorldFrame());
         contactPointProjectors[i].getSurfaceNormal().changeFrame(ReferenceFrame.getWorldFrame());

         contactPointPositions[i].set(contactPointProjectors[i].getSurfacePoint());
         scaledSurfaceNormal[i].set(contactPointProjectors[i].getSurfaceNormal());
         scaledSurfaceNormal[i].scale(contactPointProbabilities[i].getDoubleValue());
      }
   }

   private Vector3D generateRandomOffset()
   {
      // perform sampling in spherical coordinates along a hemisphere
      double offsetRadius = particleMotionStandardDeviation.getValue() * Math.abs(random.nextGaussian());
      double phi = 2.0 * Math.PI * random.nextDouble();
      double theta = Math.acos(random.nextDouble());

      double offsetX = offsetRadius * Math.cos(phi) * Math.sin(theta);
      double offsetY = offsetRadius * Math.sin(phi) * Math.sin(theta);
      double offsetZ = offsetRadius * Math.cos(theta);

      return new Vector3D(offsetX, offsetY, offsetZ);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
