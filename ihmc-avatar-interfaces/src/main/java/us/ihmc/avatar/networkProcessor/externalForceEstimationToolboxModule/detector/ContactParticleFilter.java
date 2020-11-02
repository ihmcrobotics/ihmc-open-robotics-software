package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.EstimatorContactPoint;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ForceEstimatorDynamicMatrixUpdater;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.JointspaceExternalContactEstimator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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

   private final YoDouble maximumSampleStandardDeviation = new YoDouble("maximumSampleStandardDeviation", registry);
   private final YoDouble minimumSampleStandardDeviation = new YoDouble("minimumSampleStandardDeviation", registry);
   private final YoDouble upperMotionBoundForSamplingAdjustment = new YoDouble("upperMotionBoundForSamplingAdjustment", registry);

   private final JointBasics[] joints;
   private final int dofs;
   private final JointspaceExternalContactEstimator jointspaceExternalContactEstimator;
   private final DMatrixRMaj systemJacobian;

   private final ContactPointEvaluator contactPointEvaluator = new ContactPointEvaluator();
   private final ContactPointProjector contactPointProjector;

   private final EstimatorContactPoint[] contactPoints = new EstimatorContactPoint[numberOfParticles];
   private final YoDouble[] contactPointProbabilities = new YoDouble[numberOfParticles];
   private final FramePoint3D[] sampledContactPositions = new FramePoint3D[numberOfParticles];
   private final YoFramePoint3D[] contactPointPositions = new YoFramePoint3D[numberOfParticles];
   private final YoFrameVector3D[] scaledSurfaceNormal = new YoFrameVector3D[numberOfParticles];

   private final FramePoint3D previousEstimatedContactPosition = new FramePoint3D();
   private final FramePoint3D estimatedContactPosition = new FramePoint3D();
   private final FrameVector3D estimatedContactNormal = new FrameVector3D();
   private final YoFramePoint3D averageParticlePosition = new YoFramePoint3D("averageParticlePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoEstimatedContactPosition = new YoFramePoint3D("estimatedContactPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoEstimatedContactNormal = new YoFrameVector3D("estimatedContactNormal", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble estimatedContactPositionMovement = new YoDouble("estimatedContactPositionMovement", registry);

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
      this.contactPointProjector = new ContactPointProjector(collidables);

      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPointProbabilities[i] = new YoDouble("cpProb_" + i, registry);
         contactPointPositions[i] = new YoFramePoint3D("cpPosition_" + i, ReferenceFrame.getWorldFrame(), registry);
         scaledSurfaceNormal[i] = new YoFrameVector3D("cpNorm_" + i, ReferenceFrame.getWorldFrame(), registry);
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

         YoGraphicPosition averageParticlePositionGraphic = new YoGraphicPosition("averageParticleViz", averageParticlePosition, 0.025, YoAppearance.Brown());
         YoGraphicPosition estimatedContactPositionGraphic = new YoGraphicPosition("estimatedCPViz", yoEstimatedContactPosition, 0.025, YoAppearance.Green());
         YoGraphicVector estimatedContactNormalGraphic = new YoGraphicVector("estimatedCPNormalViz", yoEstimatedContactPosition, yoEstimatedContactNormal, 0.07, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic(name, averageParticlePositionGraphic);
         graphicsListRegistry.registerYoGraphic(name, estimatedContactPositionGraphic);
         graphicsListRegistry.registerYoGraphic(name, estimatedContactNormalGraphic);
      }

      coefficientOfFriction.set(0.5);
      maximumSampleStandardDeviation.set(0.1);
      minimumSampleStandardDeviation.set(0.005);
      upperMotionBoundForSamplingAdjustment.set(0.1);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void setLinkToEstimate(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
      this.contactPointProjector.initialize(rigidBody);

      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPoints[i] = new EstimatorContactPoint(joints, rigidBody, true);
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

         contactPointProjector.computeProjection(pointToProject, contactPoints[i].getContactPointPosition(), contactPoints[i].getSurfaceNormal());
         contactPoints[i].update();
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
                                                                                     contactPoints[i].getContactPointFrame());
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
         EstimatorContactPoint contactPointToSample = contactPoints[resampleIndex];

         boolean foundPointOutsideMesh = false;
         while (!foundPointOutsideMesh)
         {
            Vector3D offset = generateSamplingOffset();
            FramePoint3D sampledContactPosition = sampledContactPositions[i];
            sampledContactPosition.setIncludingFrame(contactPointToSample.getContactPointFrame(), offset);

            if (!contactPointProjector.isPointInside(sampledContactPosition))
            {
               foundPointOutsideMesh = true;
            }
         }
      }

      // Project to surface
      for (int i = 0; i < numberOfParticles; i++)
      {
         contactPointProjector.computeProjection(sampledContactPositions[i], contactPoints[i].getContactPointPosition(), contactPoints[i].getSurfaceNormal());
         contactPoints[i].update();

         contactPoints[i].getContactPointPosition().changeFrame(ReferenceFrame.getWorldFrame());
         contactPoints[i].getSurfaceNormal().changeFrame(ReferenceFrame.getWorldFrame());

         contactPointPositions[i].set(contactPoints[i].getContactPointPosition());
         scaledSurfaceNormal[i].set(contactPoints[i].getSurfaceNormal());
         scaledSurfaceNormal[i].scale(contactPointProbabilities[i].getDoubleValue());
      }

      // Recover average contact location
      previousEstimatedContactPosition.setIncludingFrame(estimatedContactPosition);

      estimatedContactPosition.setToZero(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < numberOfParticles; i++)
      {
         estimatedContactPosition.add(contactPointPositions[i]);
      }
      estimatedContactPosition.scale(1.0 / numberOfParticles);
      averageParticlePosition.set(estimatedContactPosition);

      performBinarySearchProjection();

      estimatedContactPosition.changeFrame(ReferenceFrame.getWorldFrame());
      estimatedContactNormal.changeFrame(ReferenceFrame.getWorldFrame());
      yoEstimatedContactPosition.set(estimatedContactPosition);
      yoEstimatedContactNormal.set(estimatedContactNormal);

      estimatedContactPositionMovement.set(estimatedContactPosition.distance(previousEstimatedContactPosition));
   }

   /**
    * Projects point to mesh that might be inside the mesh. Since a joint might be composed of many meshes a simple projection isn't possible.
    * Instead, a binary search is performed along a line with constant polar/azimuth in spherical coordinates in body-fixed frame.
    */
   private void performBinarySearchProjection()
   {
      Vector3D sphericalLowerBound = new Vector3D();
      Vector3D sphericalUpperBound = new Vector3D();
      Vector3D sphericalQuery = new Vector3D();

      estimatedContactPosition.changeFrame(rigidBody.getBodyFixedFrame());
      transformToSphericalCoordinates(estimatedContactPosition, sphericalQuery);

      boolean initialPointInside = contactPointProjector.isPointInside(estimatedContactPosition);
      if (initialPointInside)
         sphericalLowerBound.set(sphericalQuery);
      else
         sphericalUpperBound.set(sphericalQuery);

      while (true)
      {
         sphericalQuery.setX(sphericalQuery.getX() * (initialPointInside ? 2.0 : 0.5));

         estimatedContactPosition.changeFrame(rigidBody.getBodyFixedFrame());
         transformToCartesianCoordinates(sphericalQuery, estimatedContactPosition);

         if (contactPointProjector.isPointInside(estimatedContactPosition) != initialPointInside)
         {
            if (initialPointInside)
               sphericalUpperBound.set(sphericalQuery);
            else
               sphericalLowerBound.set(sphericalQuery);

            break;
         }
      }

      int iterations = 8;
      for (int i = 0; i < iterations; i++)
      {
         sphericalQuery.setX(0.5 * (sphericalLowerBound.getX() + sphericalUpperBound.getX()));
         estimatedContactPosition.changeFrame(rigidBody.getBodyFixedFrame());
         transformToCartesianCoordinates(sphericalQuery, estimatedContactPosition);

         if (contactPointProjector.isPointInside(estimatedContactPosition))
            sphericalLowerBound.set(sphericalQuery);
         else
            sphericalUpperBound.set(sphericalQuery);
      }

      estimatedContactPosition.changeFrame(rigidBody.getBodyFixedFrame());
      transformToCartesianCoordinates(sphericalUpperBound, estimatedContactPosition);
   }

   private Vector3D generateSamplingOffset()
   {
      double clampedEstimatedPositionMotion = EuclidCoreTools.clamp(estimatedContactPositionMovement.getDoubleValue(),
                                                                    0.0,
                                                                    upperMotionBoundForSamplingAdjustment.getDoubleValue());
      double standardDeviation = EuclidCoreTools.interpolate(minimumSampleStandardDeviation.getDoubleValue(),
                                                             maximumSampleStandardDeviation.getDoubleValue(),
                                                             clampedEstimatedPositionMotion / upperMotionBoundForSamplingAdjustment.getDoubleValue());

      // perform sampling in spherical coordinates along a hemisphere
      double offsetRadius = standardDeviation * Math.abs(random.nextGaussian());
      double theta = 2.0 * Math.PI * random.nextDouble();
      double phi = Math.acos(random.nextDouble());

      double offsetX = offsetRadius * Math.cos(theta) * Math.sin(phi);
      double offsetY = offsetRadius * Math.sin(theta) * Math.sin(phi);
      double offsetZ = offsetRadius * Math.cos(phi);

      return new Vector3D(offsetX, offsetY, offsetZ);
   }

   private static void transformToSphericalCoordinates(Tuple3DReadOnly cartesianCoordinates, Tuple3DBasics sphericalCoordinates)
   {
      double r = EuclidCoreTools.norm(cartesianCoordinates.getX(), cartesianCoordinates.getY(), cartesianCoordinates.getZ());
      double theta = Math.atan2(cartesianCoordinates.getY(), cartesianCoordinates.getX());
      double phi = Math.acos(cartesianCoordinates.getZ() / r);

      sphericalCoordinates.set(r, theta, phi);
   }

   private static void transformToCartesianCoordinates(Tuple3DReadOnly sphericalCoordinates, Tuple3DBasics cartesianCoordinates)
   {
      double r = sphericalCoordinates.getX();
      double theta = sphericalCoordinates.getY();
      double phi = sphericalCoordinates.getZ();

      double x = r * Math.cos(theta) * Math.sin(phi);
      double y = r * Math.sin(theta) * Math.sin(phi);
      double z = r * Math.cos(phi);

      cartesianCoordinates.set(x, y, z);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
