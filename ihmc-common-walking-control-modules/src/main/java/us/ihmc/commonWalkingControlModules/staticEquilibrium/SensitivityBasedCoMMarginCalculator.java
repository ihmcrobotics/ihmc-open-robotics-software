package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator.*;
import static us.ihmc.convexOptimization.linearProgram.LinearProgramSolver.computeSensitivity;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TEN_MILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds;

public class SensitivityBasedCoMMarginCalculator
{
   private static final boolean doRandomSampling = false;
   private static final boolean doWeightingByVertexProximity = true;
   private static final double integrationDT = 1.0e-3;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /* Debug mode to compare with random sampling */
   private static final Random randomSeed = new Random(3290);
   private static final int numberOfRandomSamples = 4000000;

   private final YoRegistry randomSampleRegistry = new YoRegistry("randomSampleCoMPostureCalculator");
   private final DMatrixRMaj nullspaceRandomSample = new DMatrixRMaj(0);
   private final DMatrixRMaj randomSampleWholeBodyVelocity = new DMatrixRMaj(0);
   private final DMatrixRMaj optimalRandomSampleWholeBodyVelocity = new DMatrixRMaj(0);
   private final YoDouble yoRandomSampleSensitivity = new YoDouble("randomSampleSensitivity", randomSampleRegistry);

   private final WholeBodyContactNullspaceCalculator nullspaceCalculator;
   private final CenterOfMassStabilityMarginRegionCalculator stabilityMarginRegionCalculator;
   private final PostureConstraintMatrixVariationCalculator postureConstraintVariationCalculator;
   private final ContactPointConstraintMatrixVariation contactPointConstraintMatrixVariation;

   private final YoFramePoint2D comMarginPoint = new YoFramePoint2D("comMarginPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2D comMarginDirectionPoint = new YoFramePoint2D("comMarginDirectionPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2D comMarginDirection = new YoFrameVector2D("comMarginDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoBoolean foundSolution = new YoBoolean("foundSolution", registry);
   private final YoDouble percentage = new YoDouble("percentage", registry);
   private final YoInteger nullspaceDimensionality = new YoInteger("nullspaceDimensionality", registry);

   private final ExecutionTimer nullspaceCalculationTimer = new ExecutionTimer("nullspaceCalculationTimer", registry);

   private final YoDouble[] yoComputedSensitivity = new YoDouble[20];
   private final YoDouble[] yoVertexSensitivity = new YoDouble[DIRECTIONS_TO_OPTIMIZE];
   private final YoDouble[] yoVertexSensitivityPrev = new YoDouble[DIRECTIONS_TO_OPTIMIZE];
   private final DMatrixRMaj computedSensitivity = new DMatrixRMaj(0);
   private final DMatrixRMaj optimizedWholeBodyVelocity = new DMatrixRMaj(0);
   private final YoDouble[] yoOptimizedWholeBodyVelocity;

   private final DMatrixRMaj nullspaceVelocity = new DMatrixRMaj(0);
   private final DMatrixRMaj tempSensitivityMatrix = new DMatrixRMaj(0);

   private final YoBoolean updateNullspace = new YoBoolean("updateNullspace", registry);
   private final YoInteger sensitivityUpdatesPerTick = new YoInteger("sensitivityUpdatesPerTick", registry);
   private final YoInteger sensitivityIndexCounter = new YoInteger("sensitivityIndexCounter", registry);

   private final YoDouble yoContactPointSensitivity = new YoDouble("contactPointSensitivity", registry);
   private final YoDouble yoPostureSensitivity = new YoDouble("postureSensitivity", registry);

   /* Lowest-margin support region data */
   private int lowestMarginEdgeIndex;
   private int vertexIndexA;
   private int vertexIndexB;
   private double cosA;
   private double cosB;
   private double vertexAWeight;
   private double vertexBWeight;
   private DMatrixRMaj primalSolutionA;
   private DMatrixRMaj dualSolutionA;
   private DMatrixRMaj primalSolutionB;
   private DMatrixRMaj dualSolutionB;

   public SensitivityBasedCoMMarginCalculator(ReferenceFrame centerOfMassFrame,
                                              FullHumanoidRobotModel fullRobotModel,
                                              WholeBodyContactState wholeBodyContactState,
                                              CenterOfMassStabilityMarginRegionCalculator stabilityMarginRegionCalculator,
                                              YoRegistry parentRegistry)
   {
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.nullspaceCalculator = new WholeBodyContactNullspaceCalculator(controlledJoints, centerOfMassFrame, fullRobotModel);
      this.stabilityMarginRegionCalculator = stabilityMarginRegionCalculator;
      this.postureConstraintVariationCalculator = new PostureConstraintMatrixVariationCalculator(fullRobotModel,
                                                                                                 wholeBodyContactState,
                                                                                                 stabilityMarginRegionCalculator.getOptimizationModule(),
                                                                                                 integrationDT,
                                                                                                 registry);
      this.contactPointConstraintMatrixVariation = new ContactPointConstraintMatrixVariation(fullRobotModel,
                                                                                             wholeBodyContactState,
                                                                                             stabilityMarginRegionCalculator.getOptimizationModule());

      for (int i = 0; i < yoComputedSensitivity.length; i++)
      {
         yoComputedSensitivity[i] = new YoDouble("sensitivity" + i, registry);
      }
      for (int i = 0; i < yoVertexSensitivity.length; i++)
      {
         yoVertexSensitivity[i] = new YoDouble("sensitivityVertex" + i, registry);
         yoVertexSensitivityPrev[i] = new YoDouble("sensitivityVertexPrev" + i, registry);
      }

      yoOptimizedWholeBodyVelocity = new YoDouble[SpatialVectorReadOnly.SIZE + wholeBodyContactState.getNumberOfJoints()];

      String prefix = "qd_postOpt_";
      yoOptimizedWholeBodyVelocity[0] = new YoDouble(prefix + "PelvisAngX", registry);
      yoOptimizedWholeBodyVelocity[1] = new YoDouble(prefix + "PelvisAngY", registry);
      yoOptimizedWholeBodyVelocity[2] = new YoDouble(prefix + "PelvisAngZ", registry);
      yoOptimizedWholeBodyVelocity[3] = new YoDouble(prefix + "PelvisLinX", registry);
      yoOptimizedWholeBodyVelocity[4] = new YoDouble(prefix + "PelvisLinY", registry);
      yoOptimizedWholeBodyVelocity[5] = new YoDouble(prefix + "PelvisLinZ", registry);

      for (int i = 0; i < wholeBodyContactState.getNumberOfJoints(); i++)
      {
         yoOptimizedWholeBodyVelocity[SpatialVectorReadOnly.SIZE + i] = new YoDouble(prefix + wholeBodyContactState.getOneDoFJoints()[i].getName(), registry);
      }

      sensitivityUpdatesPerTick.set(4);
      parentRegistry.addChild(registry);

      if (doRandomSampling)
      {
         registry.addChild(randomSampleRegistry);
      }
   }

   public void initialize()
   {
      updateNullspace.set(true);
      sensitivityIndexCounter.set(0);
      foundSolution.set(false);
   }

   public boolean updateAll()
   {
      updateNullspace();

      if (!updateStabilityMarginData())
         return false;

      /* Copy nominal actuation constraint matrix */
      postureConstraintVariationCalculator.initializeFiniteDifference();

      for (int i = 0; i < nullspaceDimensionality.getValue(); i++)
      {
         sensitivityIndexCounter.set(i);
         updateSensitivity();
      }

      /* Set initial joint state and update frames */
      postureConstraintVariationCalculator.resetToInitialJointState();

      double optimalSensitivity = 0.0;
      for (int j = 0; j < nullspaceDimensionality.getValue(); j++)
      {
         optimalSensitivity += EuclidCoreTools.square(yoComputedSensitivity[j].getValue());
      }
      yoPostureSensitivity.set(Math.sqrt(optimalSensitivity));

      CommonOps_DDRM.mult(nullspaceCalculator.getNullspace(), computedSensitivity, optimizedWholeBodyVelocity);
      foundSolution.set(normalize(optimizedWholeBodyVelocity));

      for (int i = 0; i < yoOptimizedWholeBodyVelocity.length; i++)
      {
         yoOptimizedWholeBodyVelocity[i].set(foundSolution.getValue() ? optimizedWholeBodyVelocity.get(i) : Double.NaN);
      }

      return foundSolution.getValue();
   }

   public boolean updateIncremental()
   {
      if (!stabilityMarginRegionCalculator.hasSolvedWholeRegion())
      {
         return false;
      }

      if (updateNullspace.getValue())
      {
         updateNullspace.set(false);
         updateNullspace();
      }
      else
      {
         if (!updateStabilityMarginData())
            return false;

         /* Copy nominal actuation constraint matrix */
         postureConstraintVariationCalculator.initializeFiniteDifference();

         for (int i = 0; i < sensitivityUpdatesPerTick.getValue(); i++)
         {
            updateSensitivity();
            sensitivityIndexCounter.increment();

            if (sensitivityIndexCounter.getValue() >= nullspaceDimensionality.getValue())
            {
               // Done updating sensitivity values
               double optimalSensitivity = 0.0;
               for (int j = 0; j < nullspaceDimensionality.getValue(); j++)
               {
                  optimalSensitivity += EuclidCoreTools.square(yoComputedSensitivity[j].getValue());
               }
               yoPostureSensitivity.set(Math.sqrt(optimalSensitivity));

               CommonOps_DDRM.mult(nullspaceCalculator.getNullspace(), computedSensitivity, optimizedWholeBodyVelocity);
               foundSolution.set(normalize(optimizedWholeBodyVelocity));

               // Reset update flags
               updateNullspace.set(true);
               sensitivityIndexCounter.set(0);
               break;
            }
         }

         /* Set initial joint state and update frames */
         postureConstraintVariationCalculator.resetToInitialJointState();
      }

      return foundSolution.getValue();
   }

   public void updateNullspace()
   {
      /* Update contact nullspace */
      nullspaceCalculationTimer.startMeasurement();
      nullspaceCalculator.compute();
      DMatrixRMaj contactNullspace = nullspaceCalculator.getNullspace();
      nullspaceCalculationTimer.stopMeasurement();

      nullspaceVelocity.reshape(contactNullspace.getNumRows(), 1);
      computedSensitivity.reshape(contactNullspace.getNumCols(), 1);
      nullspaceRandomSample.reshape(contactNullspace.getNumCols(), 1);
      nullspaceDimensionality.set(contactNullspace.getNumCols());
   }

   public boolean updateSensitivity()
   {
      DMatrixRMaj nullspace = nullspaceCalculator.getNullspace();
      int nullspaceIndex = sensitivityIndexCounter.getValue();

      /* Extract nullspace velocity i */
      for (int row_i = 0; row_i < nullspace.getNumRows(); row_i++)
      {
         nullspaceVelocity.set(row_i, 0, nullspace.get(row_i, nullspaceIndex));
      }

      /* Compute constraint matrix variation */
      DMatrixRMaj solverConstraintVariation = postureConstraintVariationCalculator.computeFiniteDifference(nullspaceVelocity);
      double sensitivityA = cosA * computeSensitivity(solverConstraintVariation, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
      double sensitivityB = cosB * computeSensitivity(solverConstraintVariation, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
      double sensitivity = sensitivityA * vertexAWeight + sensitivityB * vertexBWeight;

      yoComputedSensitivity[nullspaceIndex].set(sensitivity);
      computedSensitivity.set(nullspaceIndex, 0, sensitivity);
      return true;
   }

   private boolean updateStabilityMarginData()
   {
      lowestMarginEdgeIndex = stabilityMarginRegionCalculator.getLowestMarginEdgeIndex();

      vertexIndexA = getVertexAOfEdge(lowestMarginEdgeIndex);
      vertexIndexB = getVertexBOfEdge(lowestMarginEdgeIndex);
      FramePoint2DReadOnly vertexA = stabilityMarginRegionCalculator.getOptimizedVertex(vertexIndexA);
      FramePoint2DReadOnly vertexB = stabilityMarginRegionCalculator.getOptimizedVertex(vertexIndexB);

      double edgeDX = vertexB.getX() - vertexA.getX();
      double edgeDY = vertexB.getY() - vertexA.getY();
      comMarginDirection.set(edgeDY, -edgeDX);
      comMarginDirection.normalize();

      FramePoint3DReadOnly centerOfMass = stabilityMarginRegionCalculator.getCenterOfMass();

      /* Intersection point along the edge is p = percentage * vertexA + (1.0 - percentage) * vertexB */
      double percentage = percentageOfIntersectionBetweenTwoLine2Ds(vertexA.getX(),
                                                                    vertexA.getY(),
                                                                    edgeDX,
                                                                    edgeDY,
                                                                    centerOfMass.getX(),
                                                                    centerOfMass.getY(),
                                                                    comMarginDirection.getX(),
                                                                    comMarginDirection.getY());
      this.percentage.set(percentage);

      comMarginPoint.interpolate(vertexA, vertexB, percentage);
      comMarginDirectionPoint.scaleAdd(0.07, comMarginDirection, comMarginPoint);

      if (Double.isNaN(percentage) || percentage < 0.0 - ONE_TEN_MILLIONTH || percentage > 1.0 + ONE_TEN_MILLIONTH)
      {
         return false;
      }

      if (doWeightingByVertexProximity)
      {
         vertexAWeight = 1.0 - percentage;
         vertexBWeight = percentage;
      }
      else
      {
         vertexAWeight = 0.5;
         vertexBWeight = 0.5;
      }

      cosA = comMarginDirection.getX() * queryDirectionX(vertexIndexA) + comMarginDirection.getY() * queryDirectionY(vertexIndexA);
      cosB = comMarginDirection.getX() * queryDirectionX(vertexIndexB) + comMarginDirection.getY() * queryDirectionY(vertexIndexB);

      primalSolutionA = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexA);
      dualSolutionA = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexA);

      primalSolutionB = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexB);
      dualSolutionB = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexB);

      return true;
   }

   private void doRandomSamplingPostureAdjustments(DMatrixRMaj contactNullspace,
                                                   double cosA,
                                                   double cosB,
                                                   DMatrixRMaj primalSolutionA,
                                                   DMatrixRMaj dualSolutionA,
                                                   DMatrixRMaj primalSolutionB,
                                                   DMatrixRMaj dualSolutionB)
   {
      double bestRandomSensitivity = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRandomSamples; i++)
      {
         if (i % 1000 == 0)
            System.out.println(i + " / " + numberOfRandomSamples);

         for (int j = 0; j < nullspaceRandomSample.getNumRows(); j++)
         {
            nullspaceRandomSample.set(j, EuclidCoreRandomTools.nextDouble(randomSeed, 1.0));
         }

         CommonOps_DDRM.mult(contactNullspace, nullspaceRandomSample, randomSampleWholeBodyVelocity);
         if (!normalize(randomSampleWholeBodyVelocity))
            continue;
         DMatrixRMaj solverConstraintVariationRand = postureConstraintVariationCalculator.computeFiniteDifference(randomSampleWholeBodyVelocity);

         double sensitivityRandA = cosA * computeSensitivity(solverConstraintVariationRand, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
         double sensitivityRandB = cosB * computeSensitivity(solverConstraintVariationRand, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
         double sensitivityRand = sensitivityRandA * vertexAWeight + sensitivityRandB * vertexBWeight;

         if (sensitivityRand > bestRandomSensitivity)
         {
            bestRandomSensitivity = sensitivityRand;
            optimalRandomSampleWholeBodyVelocity.set(randomSampleWholeBodyVelocity);
         }
      }

      yoRandomSampleSensitivity.set(bestRandomSensitivity);
   }

   public boolean computeContactPointSensitivity(int contactPointIndex, Vector3DReadOnly contactPointAdjustment)
   {
      if (!stabilityMarginRegionCalculator.hasSolvedWholeRegion())
         return false;

      if (!updateStabilityMarginData())
         return false;

      int lowestMarginEdgeIndex = stabilityMarginRegionCalculator.getLowestMarginEdgeIndex();

      int vertexIndexA = getVertexAOfEdge(lowestMarginEdgeIndex);
      int vertexIndexB = getVertexBOfEdge(lowestMarginEdgeIndex);

      DMatrixRMaj primalSolutionA = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexA);
      DMatrixRMaj dualSolutionA = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexA);

      DMatrixRMaj primalSolutionB = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexB);
      DMatrixRMaj dualSolutionB = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexB);

      DMatrixRMaj constraintMatrixVariation = contactPointConstraintMatrixVariation.compute(contactPointIndex, contactPointAdjustment);
      double sensitivityA = computeSensitivity(constraintMatrixVariation, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
      double sensitivityB = computeSensitivity(constraintMatrixVariation, primalSolutionB, dualSolutionB, tempSensitivityMatrix);

      yoContactPointSensitivity.set(sensitivityA * vertexAWeight + sensitivityB * vertexBWeight);
      return true;
   }

   public double getPostureSensitivity()
   {
      return yoPostureSensitivity.getValue();
   }

   public double getContactPointSensitivity()
   {
      return yoContactPointSensitivity.getValue();
   }

   private static boolean normalize(DMatrixRMaj matrix)
   {
      if (!MatrixFeatures_DDRM.isVector(matrix))
         return false;

      double magnitudeSq = 0.0;
      for (int i = 0; i < matrix.getNumRows(); i++)
      {
         magnitudeSq += EuclidCoreTools.square(matrix.get(i));
      }

      if (magnitudeSq < 1.0e-10)
         return false;

      CommonOps_DDRM.scale(1.0 / Math.sqrt(magnitudeSq), matrix);
      return true;
   }

   public boolean foundSolution()
   {
      return foundSolution.getValue();
   }

   private static double computeMarginToJointLimit(OneDoFJointBasics joint)
   {
      return Math.min(Math.abs(joint.getQ() - joint.getJointLimitLower()), Math.abs(joint.getQ() - joint.getJointLimitUpper()));
   }

   public DMatrixRMaj getOptimizedWholeBodyVelocity()
   {
      return optimizedWholeBodyVelocity;
   }

   public void addJointToIgnore(OneDoFJointBasics jointToIgnore)
   {
      nullspaceCalculator.addJointToIgnore(jointToIgnore);
   }

   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition(getClass().getSimpleName());
      groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("comMarginPositionGraphic", comMarginPoint, 0.003, ColorDefinitions.Blue(), DefaultPoint2DGraphic.DIAMOND));
      groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition("comMarginDirectionGraphic", comMarginPoint,
                                                                                              comMarginDirectionPoint, ColorDefinitions.Blue()));
      return groupDefinition;
   }
}
