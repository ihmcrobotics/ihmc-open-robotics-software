package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import static us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator.*;
import static us.ihmc.convexOptimization.linearProgram.LinearProgramSolver.computeSensitivity;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TEN_MILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds;

/**
 * For a stability region computed by {@link StabilityMarginRegionCalculator}, where the margin m
 * is the distance from the CoM to the nearest point on the region boundary at joint configuration q.
 * This computes the gradient of m(q), assuming fixed contacts and a fixed CoM xy position.
 */
public class SensitivityBasedStabilityGradientCalculator
{
   private static final double integrationDT = 1.0e-3;
   private static final int XY_DIMENSIONS = 2;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /* The sensitivity analysis assumes a constant CoM xy position and contact points. This computes the nullspace of those constraints */
   private final ContactNullspaceCalculator nullspaceCalculator;
   /* Stability margin calculator. The data on the closest edge is used to compute the margin gradient */
   private final StabilityMarginRegionCalculator stabilityMarginRegionCalculator;
   /* Compute dA/dt for a given q, qd, where A is the constraint matrix in the LP computed by StabilityMarginRegionCalculator */
   private final PostureConstraintMatrixVariationCalculator postureConstraintVariationCalculator;

   /* Point p on the region boundary closest to the CoM. The margin m is defined as m = |c - p|, where c is the CoM */
   private final YoFramePoint2D comMarginPoint = new YoFramePoint2D("comMarginPoint", ReferenceFrame.getWorldFrame(), registry);
   /* Added for graphical reasons */
   private final YoFramePoint2D comMarginDirectionPoint = new YoFramePoint2D("comMarginDirectionPoint", ReferenceFrame.getWorldFrame(), registry);
   /* The normalized vector from c to p, where c is the CoM and p is comMarginPoint */
   private final YoFrameVector2D yoStabilityMarginDirection = new YoFrameVector2D("stabilityMarginDirection", ReferenceFrame.getWorldFrame(), registry);
   /* Dimensionality of the contact nullspace */
   private final YoInteger nullspaceDimensionality = new YoInteger("nullspaceDimensionality", registry);
   /* Timer for nullspace calculation */
   private final ExecutionTimer nullspaceCalculationTimer = new ExecutionTimer("nullspaceCalculationTimer", registry);

   /* Sensitivity of margin along qd_i in nullspace: dm/dt(qd_i), where qd_i is normalized */
   private final DMatrixRMaj computedSensitivity = new DMatrixRMaj(0);
   /* Gradient of the comMarginPoint with respect to q (the robot's configuration) along the line stabilityMarginDirection */
   private final DMatrixRMaj stabilityBoundaryGradient = new DMatrixRMaj(0);
   /* The gradient of the com along the line stabilityMarginDirection */
   private final DMatrixRMaj comGradient = new DMatrixRMaj(0);
   /* Normalized stability boundary gradient */
   private final DMatrixRMaj normalizedStabilityBoundaryGradient = new DMatrixRMaj(0);

   /* The normalized vector from c to p, where c is the CoM and p is comMarginPoint */
   private final DMatrixRMaj stabilityMarginDirection = new DMatrixRMaj(XY_DIMENSIONS, 1);
   /* XY component of the centroidal momentum matrix, used to compute the CoM component of the objective term */
   private final DMatrixRMaj centroidalMomentumMatrixXY = new DMatrixRMaj(0);
   /* The gradient of the stability margin m, s.t. dm/dt = v · grad(m), where grad(m) is this value */
   private final DMatrixRMaj stabilityMarginGradient = new DMatrixRMaj(0);
   /* Total robot mass, used to compute the CoM linear velocity */
   private final double robotMass;

   /* Nullspace velocity column qd_i */
   private final DMatrixRMaj nullspaceVelocity = new DMatrixRMaj(0);
   /* Temp field used to call LinearProgramSolver#computeSensitivity */
   private final DMatrixRMaj tempSensitivityMatrix = new DMatrixRMaj(0);

   /* Postural sensitivity, given as |N grad m(q)|, where N is the nullspace projector */
   private final YoDouble yoPostureSensitivity = new YoDouble("postureSensitivity", registry);
   /* YoVariable data of stability margin gradient */
   private final YoDouble[] yoStabilityMarginGradient;

   /* To compute the CoM component of the margin objective */
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;

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

   private final TIntArrayList jointsToIgnore = new TIntArrayList();

   public SensitivityBasedStabilityGradientCalculator(FullHumanoidRobotModel fullRobotModel,
                                                      WholeBodyContactState wholeBodyContactState,
                                                      StabilityMarginRegionCalculator stabilityMarginRegionCalculator,
                                                      YoRegistry parentRegistry)
   {
      this.nullspaceCalculator = new ContactNullspaceCalculator(fullRobotModel, wholeBodyContactState, registry);
      this.stabilityMarginRegionCalculator = stabilityMarginRegionCalculator;
      this.postureConstraintVariationCalculator = new PostureConstraintMatrixVariationCalculator(fullRobotModel,
                                                                                                 wholeBodyContactState,
                                                                                                 stabilityMarginRegionCalculator.getOptimizationModule(),
                                                                                                 integrationDT,
                                                                                                 registry);
      robotMass = fullRobotModel.getTotalMass();

      JointBasics[] controlledJoints = computeJointsToOptimizeFor(fullRobotModel);
      OneDoFJointBasics[] controllableOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(multiBodySystemInput, ReferenceFrame.getWorldFrame());

      yoStabilityMarginGradient = new YoDouble[Twist.SIZE + controllableOneDoFJoints.length];
      String namePrefix = "stabilityMarginGrad_";

      for (int i = 0; i < yoStabilityMarginGradient.length; i++)
      {
         if (i < Twist.SIZE)
         {
            int numAxes = Axis3D.values().length;
            Axis3D axis = Axis3D.values()[i % numAxes];
            String prefix = i < numAxes ? "w" : "v";
            yoStabilityMarginGradient[i] = new YoDouble(namePrefix + prefix + axis, registry);
         }
         else
         {
            String jointName = controllableOneDoFJoints[i - Twist.SIZE].getName();
            yoStabilityMarginGradient[i] = new YoDouble(namePrefix + jointName, registry);
         }
      }

      jointsToIgnore.addAll(multiBodySystemInput.getJointMatrixIndexProvider().getJointDoFIndices(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL)));
      jointsToIgnore.addAll(multiBodySystemInput.getJointMatrixIndexProvider().getJointDoFIndices(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)));

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      updateNullspace();

      int lowestMarginEdgeIndex = stabilityMarginRegionCalculator.getLowestMarginEdgeIndex();
      if (!updateStabilityMarginData(lowestMarginEdgeIndex))
         return;

      /* Copy nominal actuation constraint matrix */
      postureConstraintVariationCalculator.initializeFiniteDifference();

      for (int nullspaceIndex = 0; nullspaceIndex < nullspaceDimensionality.getValue(); nullspaceIndex++)
      {
         updateSensitivity(nullspaceIndex);
      }

      /* Set initial joint state and update frames */
      postureConstraintVariationCalculator.resetToInitialJointState();

      double optimalSensitivity = 0.0;
      for (int i = 0; i < nullspaceDimensionality.getValue(); i++)
      {
         optimalSensitivity += EuclidCoreTools.square(computedSensitivity.get(i));
      }
      yoPostureSensitivity.set(Math.sqrt(optimalSensitivity));

      CommonOps_DDRM.mult(nullspaceCalculator.getNullspace(), computedSensitivity, stabilityBoundaryGradient);

      normalizedStabilityBoundaryGradient.set(stabilityBoundaryGradient);
      if (yoPostureSensitivity.getValue() > 1.0e-5)
      {
         CommonOps_DDRM.scale(1.0 / yoPostureSensitivity.getValue(), normalizedStabilityBoundaryGradient);
      }
      else
      {
         normalizedStabilityBoundaryGradient.zero();
      }

      centroidalMomentumCalculator.reset();
      DMatrixRMaj centroidalMomentumMatrix = centroidalMomentumCalculator.getCentroidalMomentumMatrix();
      centroidalMomentumMatrixXY.reshape(XY_DIMENSIONS, centroidalMomentumMatrix.getNumCols());
      MatrixTools.setMatrixBlock(centroidalMomentumMatrixXY, 0, 0, centroidalMomentumMatrix, 3, 0, XY_DIMENSIONS, centroidalMomentumMatrix.getNumCols(), 1.0 / robotMass);

      CommonOps_DDRM.multTransA(centroidalMomentumMatrixXY, stabilityMarginDirection, comGradient);
      CommonOps_DDRM.subtract(stabilityBoundaryGradient, comGradient, stabilityMarginGradient);

      for (int i = 0; i < jointsToIgnore.size(); i++)
      {
         stabilityMarginGradient.set(jointsToIgnore.get(i), 0.0);
      }

      for (int i = 0; i < stabilityMarginGradient.getNumRows(); i++)
      {
         yoStabilityMarginGradient[i].set(stabilityMarginGradient.get(i));
      }
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
      nullspaceDimensionality.set(contactNullspace.getNumCols());
   }

   public void updateSensitivity(int nullspaceIndex)
   {
      DMatrixRMaj nullspace = nullspaceCalculator.getNullspace();

      /* Extract nullspace velocity i */
      for (int row_i = 0; row_i < nullspace.getNumRows(); row_i++)
      {
         nullspaceVelocity.set(row_i, 0, nullspace.get(row_i, nullspaceIndex));
      }

      /* Compute constraint matrix variation */
      DMatrixRMaj solverConstraintVariation = postureConstraintVariationCalculator.computeFiniteDifference(nullspaceVelocity);
      double sensitivityMultiplier = stabilityMarginRegionCalculator.getOptimizationModule().getStabilityPointGradientCoefficient();

      double sensitivityA = sensitivityMultiplier * cosA * computeSensitivity(solverConstraintVariation, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
      double sensitivityB = sensitivityMultiplier * cosB * computeSensitivity(solverConstraintVariation, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
      double sensitivity = sensitivityA * vertexAWeight + sensitivityB * vertexBWeight;
      computedSensitivity.set(nullspaceIndex, 0, sensitivity);
   }

   private boolean updateStabilityMarginData(int edgeIndex)
   {
      FrameConvexPolygon2DReadOnly feasibleRegion = stabilityMarginRegionCalculator.getFeasibleRegion();

      FramePoint2DReadOnly vertexA = feasibleRegion.getVertex(edgeIndex);
      FramePoint2DReadOnly vertexB = feasibleRegion.getNextVertex(edgeIndex);

      vertexIndexA = stabilityMarginRegionCalculator.fromEuclidIndex(edgeIndex);
      vertexIndexB = stabilityMarginRegionCalculator.fromEuclidIndex(feasibleRegion.getNextVertexIndex(edgeIndex));

      double edgeDX = vertexB.getX() - vertexA.getX();
      double edgeDY = vertexB.getY() - vertexA.getY();
      yoStabilityMarginDirection.set(-edgeDY, edgeDX);
      yoStabilityMarginDirection.normalize();
      yoStabilityMarginDirection.get(stabilityMarginDirection);

      FramePoint3DReadOnly centerOfMass = stabilityMarginRegionCalculator.getCenterOfMass();

      /* Intersection point along the edge is p = percentage * vertexA + (1.0 - percentage) * vertexB */
      double percentage = percentageOfIntersectionBetweenTwoLine2Ds(vertexA.getX(),
                                                                    vertexA.getY(),
                                                                    edgeDX,
                                                                    edgeDY,
                                                                    centerOfMass.getX(),
                                                                    centerOfMass.getY(),
                                                                    yoStabilityMarginDirection.getX(),
                                                                    yoStabilityMarginDirection.getY());

      comMarginPoint.interpolate(vertexA, vertexB, percentage);
      comMarginDirectionPoint.scaleAdd(0.07, yoStabilityMarginDirection, comMarginPoint);

      if (Double.isNaN(percentage) || percentage < 0.0 - ONE_TEN_MILLIONTH || percentage > 1.0 + ONE_TEN_MILLIONTH)
      {
         return false;
      }

      vertexAWeight = 1.0 - percentage;
      vertexBWeight = percentage;

      cosA = yoStabilityMarginDirection.getX() * queryDirectionX(vertexIndexA) + yoStabilityMarginDirection.getY() * queryDirectionY(vertexIndexA);
      cosB = yoStabilityMarginDirection.getX() * queryDirectionX(vertexIndexB) + yoStabilityMarginDirection.getY() * queryDirectionY(vertexIndexB);

      primalSolutionA = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexA);
      dualSolutionA = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexA);

      primalSolutionB = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndexB);
      dualSolutionB = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndexB);

      return true;
   }

   public double getPostureSensitivity()
   {
      return yoPostureSensitivity.getValue();
   }

   public DMatrixRMaj getNomalizedStabilityMarginGradient()
   {
      return normalizedStabilityBoundaryGradient;
   }

   public ContactNullspaceCalculator getNullspaceCalculator()
   {
      return nullspaceCalculator;
   }

   public FrameVector2DReadOnly getStabilityMarginDirection()
   {
      return yoStabilityMarginDirection;
   }

   public YoFramePoint2D getComMarginPoint()
   {
      return comMarginPoint;
   }

   /**
    * Returns the Jacobian J for which Jv = dm/dt, where v is the whole-body velocity and m is the stability margin.
    */
   public DMatrixRMaj getStabilityMarginGradient()
   {
      return stabilityMarginGradient;
   }

   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition(getClass().getSimpleName());
      groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("comMarginPositionGraphic",
                                                                              comMarginPoint,
                                                                              0.003,
                                                                              ColorDefinitions.Blue(),
                                                                              DefaultPoint2DGraphic.DIAMOND));
      groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition("comMarginDirectionGraphic",
                                                                                              comMarginPoint,
                                                                                              comMarginDirectionPoint,
                                                                                              ColorDefinitions.Blue()));
      return groupDefinition;
   }
}
