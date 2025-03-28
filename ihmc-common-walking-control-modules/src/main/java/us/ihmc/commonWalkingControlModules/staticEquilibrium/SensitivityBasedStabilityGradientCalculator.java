package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import static us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator.queryDirectionX;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator.queryDirectionY;
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
   private static final boolean APPLY_JOINT_LIMIT_FILTER = true;
   private static final boolean USE_AREA_BASED_CONTACT_ADJUSTMENT = true;
   private static final boolean USE_HEURISTIC_MARGIN = true;
   private static final double INTEGRATION_DT = 1.0e-3;
   private static final int XY_DIMENSIONS = 2;
   private static final boolean COMPUTE_EXPECTED_MARGIN_VELOCITY = true;
   private static final boolean INCLUDE_POSTURE_OBJ_IN_CONTACT_ADJUSTMENT = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /* Robot model */
   private final FullHumanoidRobotModel fullRobotModel;
   /* Description of whole-body contact state */
   private final WholeBodyContactState wholeBodyContactState;

   /* The sensitivity analysis assumes a constant CoM xy position and contact points. This computes the nullspace of those constraints */
   private final ContactNullspaceCalculator nullspaceCalculator;
   /* For computing contact point adjustment, this computes the whole-body velocity tangent to a given surface */
   private final SideDependentList<ContactTangentCalculator> contactTangentCalculators = new SideDependentList<>();
   /* Stability margin calculator. The data on the closest edge is used to compute the margin gradient */
   private final StabilityMarginRegionCalculator stabilityMarginRegionCalculator;
   /* Compute dA/dt for a given q, qd, where A is the constraint matrix in the LP computed by StabilityMarginRegionCalculator */
   private final PostureConstraintMatrixVariationCalculator postureConstraintVariationCalculator;
   /* Compute dA/dt for a given dr_i/dt, where r_i is the position of contact point i and A is the constraint matrix in the LP computed by StabilityMarginRegionCalculator */
   private final ContactPointConstraintMatrixVariationCalculator contactPointConstraintMatrixVariation;

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

   /* Postural sensitivity, given as |N grad m(q)|, for configuration q, where N is the nullspace projector */
   private final AlphaFilteredYoVariable yoPostureSensitivity;
   /* Contact sensitivity, given as |grad m(c)|, for contact point c */
   private final YoDouble yoContactSensitivity = new YoDouble("contactSensitivity", registry);
   /* YoVariable data of stability margin gradient */
   private final YoDouble[] yoStabilityMarginGradient;

   private final YoDouble expectedSensitivity = new YoDouble("expectedSensitivity", registry);
   private final YoDouble gradientFilterAlpha = new YoDouble("gradientFilterAlpha", registry);

   /* Indexed controllable joints */
   private final OneDoFJointBasics[] controllableOneDoFJoints;

   private final SideDependentList<FrameVector3D> contactPointOptimalAdjustments = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> yoContactPointOptimalAdjustmentsPoints = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> yoContactPointAreaAdjustments = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> yoContactPointMarginAdjustments = new SideDependentList<>();
   private final SideDependentList<YoDouble> yoContactAdjustmentNorm = new SideDependentList<>();

   private final FrameVector3D tempFrameVector = new FrameVector3D();
   private final FrameVector3D tempFrameVectorX = new FrameVector3D();
   private final FrameVector3D tempFrameVectorY = new FrameVector3D();
   private final Vector2D tempVector2D = new Vector2D();
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

   private final DMatrixRMaj dAX = new DMatrixRMaj(0);
   private final DMatrixRMaj dAY = new DMatrixRMaj(0);

   private final TIntArrayList jointsToIgnore = new TIntArrayList();

   public SensitivityBasedStabilityGradientCalculator(FullHumanoidRobotModel fullRobotModel,
                                                      WholeBodyContactState wholeBodyContactState,
                                                      StabilityMarginRegionCalculator stabilityMarginRegionCalculator,
                                                      CentroidalMomentumCalculator centroidalMomentumCalculator,
                                                      YoGraphicsListRegistry graphicsListRegistry,
                                                      YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.nullspaceCalculator = new ContactNullspaceCalculator(fullRobotModel, wholeBodyContactState, centroidalMomentumCalculator, registry);
      this.stabilityMarginRegionCalculator = stabilityMarginRegionCalculator;
      this.wholeBodyContactState = wholeBodyContactState;
      this.postureConstraintVariationCalculator = new PostureConstraintMatrixVariationCalculator(fullRobotModel,
                                                                                                 wholeBodyContactState,
                                                                                                 stabilityMarginRegionCalculator.getOptimizationModule(),
                                                                                                 INTEGRATION_DT,
                                                                                                 registry);
      contactPointConstraintMatrixVariation = new ContactPointConstraintMatrixVariationCalculator(stabilityMarginRegionCalculator.getOptimizationModule());
      robotMass = fullRobotModel.getTotalMass();

      JointBasics[] controlledJoints = computeJointsToOptimizeFor(fullRobotModel);
      controllableOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
//      qd = new DMatrixRMaj(6 + controllableOneDoFJoints.length, 1);

      yoStabilityMarginGradient = new YoDouble[Twist.SIZE + controllableOneDoFJoints.length];
      String namePrefix = "stabilityGrad_";

      gradientFilterAlpha.set(0.97);
      yoPostureSensitivity = new AlphaFilteredYoVariable("postureSensitivity", registry, gradientFilterAlpha);

      for (int i = 0; i < yoStabilityMarginGradient.length; i++)
      {
         if (i < Twist.SIZE)
         {
            int numAxes = Axis3D.values().length;
            Axis3D axis = Axis3D.values()[i % numAxes];
            String prefix = i < numAxes ? "w" : "v";
            String name = namePrefix + prefix + axis;
            yoStabilityMarginGradient[i] = new YoDouble(name, registry);
         }
         else
         {
            String jointName = controllableOneDoFJoints[i - Twist.SIZE].getName();
            String name = namePrefix + jointName;
            yoStabilityMarginGradient[i] = new YoDouble(name, registry);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         contactPointOptimalAdjustments.put(robotSide, new FrameVector3D());

         String prefix = robotSide.getCamelCaseNameForStartOfExpression();
         yoContactPointOptimalAdjustmentsPoints.put(robotSide, new YoFramePoint3D(prefix + "contact", ReferenceFrame.getWorldFrame(), registry));
         yoContactPointAreaAdjustments.put(robotSide, new YoFrameVector3D(prefix + "contactAreaAdj", ReferenceFrame.getWorldFrame(), registry));
         yoContactPointMarginAdjustments.put(robotSide, new YoFrameVector3D(prefix + "contactMarginAdj", ReferenceFrame.getWorldFrame(), registry));
         yoContactAdjustmentNorm.put(robotSide, new YoDouble(prefix + "contactAdjustmentNorm", registry));
         contactTangentCalculators.put(robotSide, new ContactTangentCalculator(fullRobotModel, robotSide));

         if (graphicsListRegistry != null)
         {
            YoGraphicVector contactAreaAdjustmentGraphic = new YoGraphicVector(prefix + "contactAreaAdj", yoContactPointOptimalAdjustmentsPoints.get(robotSide), yoContactPointAreaAdjustments.get(robotSide), 2.0, YoAppearance.Red());
            YoGraphicVector contactMarginAdjustmentGraphic = new YoGraphicVector(prefix + "contactMarginAdj", yoContactPointOptimalAdjustmentsPoints.get(robotSide), yoContactPointMarginAdjustments.get(robotSide), 2.0, YoAppearance.Green());
            graphicsListRegistry.registerYoGraphic("Contact adjustment", contactAreaAdjustmentGraphic);
            graphicsListRegistry.registerYoGraphic("Contact adjustment", contactMarginAdjustmentGraphic);
         }
      }

      jointsToIgnore.addAll(multiBodySystemInput.getJointMatrixIndexProvider().getJointDoFIndices(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL)));
      jointsToIgnore.addAll(multiBodySystemInput.getJointMatrixIndexProvider().getJointDoFIndices(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)));

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      yoPostureSensitivity.set(0.0);
   }

   public void computePostureAdjustment()
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

      if (COMPUTE_EXPECTED_MARGIN_VELOCITY)
         computeExpectedMarginVelocity();

      /* Set initial joint state and update frames */
      postureConstraintVariationCalculator.resetToInitialJointState();

      double optimalSensitivity = 0.0;
      for (int i = 0; i < nullspaceDimensionality.getValue(); i++)
      {
         optimalSensitivity += EuclidCoreTools.square(computedSensitivity.get(i));
      }
      yoPostureSensitivity.update(Math.sqrt(optimalSensitivity));

      CommonOps_DDRM.mult(nullspaceCalculator.getNullspace(), computedSensitivity, stabilityBoundaryGradient);

      DMatrixRMaj centroidalMomentumMatrix = nullspaceCalculator.getCentroidalMomentumCalculator().getCentroidalMomentumMatrix();
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
         yoStabilityMarginGradient[i].set(stabilityBoundaryGradient.get(i));
      }
   }

   public FrameVector3DReadOnly computeContactPointAdjustment(RobotSide robotSide)
   {
      FrameVector3D contactPointOptimalAdjustment = contactPointOptimalAdjustments.get(robotSide);
      postureConstraintVariationCalculator.initializeFiniteDifference();

      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      int contact_idx = wholeBodyContactState.indexOf(hand);

      if (INCLUDE_POSTURE_OBJ_IN_CONTACT_ADJUSTMENT)
         contactTangentCalculators.get(robotSide).computeJacobian();

      // Constraint matrix variation
      tempFrameVectorX.setIncludingFrame(wholeBodyContactState.getContactFrame(contact_idx), Axis3D.X);
      tempFrameVectorX.changeFrame(ReferenceFrame.getWorldFrame());
      dAX.set(contactPointConstraintMatrixVariation.compute(contact_idx, tempFrameVectorX));
      if (INCLUDE_POSTURE_OBJ_IN_CONTACT_ADJUSTMENT)
         CommonOps_DDRM.addEquals(dAX, postureConstraintVariationCalculator.computeFiniteDifference(contactTangentCalculators.get(robotSide).computeWholeBodyVelocity(tempFrameVectorX)));

      tempFrameVectorY.setIncludingFrame(wholeBodyContactState.getContactFrame(contact_idx), Axis3D.Y);
      tempFrameVectorY.changeFrame(ReferenceFrame.getWorldFrame());
      dAY.set(contactPointConstraintMatrixVariation.compute(contact_idx, tempFrameVectorY));
      if (INCLUDE_POSTURE_OBJ_IN_CONTACT_ADJUSTMENT)
         CommonOps_DDRM.addEquals(dAY, postureConstraintVariationCalculator.computeFiniteDifference(contactTangentCalculators.get(robotSide).computeWholeBodyVelocity(tempFrameVectorY)));

      FrameConvexPolygon2DReadOnly feasibleRegion = stabilityMarginRegionCalculator.getFeasibleRegion();
      double dAreaX = 0.0;
      double dAreaY = 0.0;

      {
         for (int vertex_idx = 0; vertex_idx < feasibleRegion.getNumberOfVertices(); vertex_idx++)
         {
            FramePoint2DReadOnly vertex = feasibleRegion.getVertex(vertex_idx);
            FramePoint2DReadOnly vertexPrev = feasibleRegion.getPreviousVertex(vertex_idx);
            FramePoint2DReadOnly vertexNext = feasibleRegion.getNextVertex(vertex_idx);
            double distance = 0.5 * (vertex.distance(vertexPrev) + vertex.distance(vertexNext));

            int vertexIndex = stabilityMarginRegionCalculator.fromEuclidIndex(vertex_idx);
            DMatrixRMaj primalSolution = stabilityMarginRegionCalculator.getSolverPrimalSolution(vertexIndex);
            DMatrixRMaj dualSolution = stabilityMarginRegionCalculator.getSolverDualSolution(vertexIndex);

            double sensitivityX = computeSensitivity(dAX, primalSolution, dualSolution, tempSensitivityMatrix);
            dAreaX += distance * sensitivityX;

            double sensitivityY = computeSensitivity(dAY, primalSolution, dualSolution, tempSensitivityMatrix);
            dAreaY += distance * sensitivityY;
         }
      }

      double dMarginX = 0.0;
      double dMarginY = 0.0;

      { // Compute via farthest edge
         // Find query direction parallel anti-parallel to normal

         int contactAdjustmentMarginIndex;

         if (USE_HEURISTIC_MARGIN)
         {
            tempFrameVector.setIncludingFrame(wholeBodyContactState.getContactFrame(contact_idx), Axis3D.Z);
            tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

            double maxDot = 0.0;
            contactAdjustmentMarginIndex = 0;

            for (int j = 0; j < feasibleRegion.getNumberOfVertices(); j++)
            {
               FramePoint2DReadOnly v0 = feasibleRegion.getVertex(j);
               FramePoint2DReadOnly v1 = feasibleRegion.getNextVertex(j);
               tempVector2D.sub(v1, v0);
               tempVector2D.set(tempVector2D.getY(), -tempVector2D.getX());
               tempVector2D.normalize();
               double dot = tempVector2D.getX() * tempFrameVector.getX() + tempVector2D.getY() * tempFrameVector.getY();
               if (dot > maxDot)
               {
                  maxDot = dot;
                  contactAdjustmentMarginIndex = j;
               }
            }
         }
         else
         {
            contactAdjustmentMarginIndex = stabilityMarginRegionCalculator.getLowestMarginEdgeIndex();
         }

         if (updateStabilityMarginData(contactAdjustmentMarginIndex))
         {
            double sensitivityAX = cosA * computeSensitivity(dAX, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
            double sensitivityBX = cosB * computeSensitivity(dAX, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
            double sensitivityX = sensitivityAX * vertexAWeight + sensitivityBX * vertexBWeight;
            dMarginX = sensitivityX;

            double sensitivityAY = cosA * computeSensitivity(dAY, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
            double sensitivityBY = cosB * computeSensitivity(dAY, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
            double sensitivityY = sensitivityAY * vertexAWeight + sensitivityBY * vertexBWeight;
            dMarginY = sensitivityY;
         }
      }

      // Update by area
      yoContactPointAreaAdjustments.get(robotSide).setToZero();
      yoContactPointAreaAdjustments.get(robotSide).scaleAdd(dAreaX, tempFrameVectorX, yoContactPointAreaAdjustments.get(robotSide));
      yoContactPointAreaAdjustments.get(robotSide).scaleAdd(dAreaY, tempFrameVectorY, yoContactPointAreaAdjustments.get(robotSide));

      // Update by margin
      yoContactPointMarginAdjustments.get(robotSide).setToZero();
      yoContactPointMarginAdjustments.get(robotSide).scaleAdd(dMarginX, tempFrameVectorX, yoContactPointMarginAdjustments.get(robotSide));
      yoContactPointMarginAdjustments.get(robotSide).scaleAdd(dMarginY, tempFrameVectorY, yoContactPointMarginAdjustments.get(robotSide));

      // Scale by sensitivity
      contactPointOptimalAdjustment.set(USE_AREA_BASED_CONTACT_ADJUSTMENT ? yoContactPointAreaAdjustments.get(robotSide) : yoContactPointMarginAdjustments.get(robotSide));

      if (USE_AREA_BASED_CONTACT_ADJUSTMENT)
      {
         yoContactSensitivity.set(EuclidCoreTools.norm(dAreaX, dAreaY));
      }
      else
      {
         yoContactSensitivity.set(EuclidCoreTools.norm(dMarginX, dMarginY));
      }

      if (INCLUDE_POSTURE_OBJ_IN_CONTACT_ADJUSTMENT)
         postureConstraintVariationCalculator.resetToInitialJointState();

      // For visualization
      yoContactPointOptimalAdjustmentsPoints.get(robotSide).setFromReferenceFrame(wholeBodyContactState.getContactFrame(contact_idx));
      return contactPointOptimalAdjustment;
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

   public void computeExpectedMarginVelocity()
   {
      DMatrixRMaj currentWholeBodyVelocity = postureConstraintVariationCalculator.getCurrentWholeBodyVelocity();
      DMatrixRMaj solverConstraintVariation = postureConstraintVariationCalculator.computeFiniteDifference(currentWholeBodyVelocity);
      double sensitivityMultiplier = stabilityMarginRegionCalculator.getOptimizationModule().getStabilityPointGradientCoefficient();

      double sensitivityA = sensitivityMultiplier * cosA * computeSensitivity(solverConstraintVariation, primalSolutionA, dualSolutionA, tempSensitivityMatrix);
      double sensitivityB = sensitivityMultiplier * cosB * computeSensitivity(solverConstraintVariation, primalSolutionB, dualSolutionB, tempSensitivityMatrix);
      expectedSensitivity.set(sensitivityA * vertexAWeight + sensitivityB * vertexBWeight);
   }

   public CentroidalMomentumCalculator getCentroidalMomentumCalculator()
   {
      return nullspaceCalculator.getCentroidalMomentumCalculator();
   }

   private void updateSensitivity(int nullspaceIndex)
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

      double jointLimitAlpha = APPLY_JOINT_LIMIT_FILTER ? computeJointLimitFilter() : 1.0;

      computedSensitivity.set(nullspaceIndex, 0, jointLimitAlpha * sensitivity);
   }

   private double computeJointLimitFilter()
   {
      int rootJointIndices = Twist.SIZE;
      double jointLimitAlpha = 1.0;

      for (int dof_idx = rootJointIndices; dof_idx < nullspaceVelocity.getNumRows(); dof_idx++)
      {
         int joint_idx = dof_idx - rootJointIndices;
         OneDoFJointBasics joint = controllableOneDoFJoints[joint_idx];

         double romFraction0 = 0.08;
         double romFraction1 = 0.04;

         double q = joint.getQ();
         double qdNullspace = nullspaceVelocity.get(dof_idx);

         double qUpper = joint.getJointLimitUpper();
         double qLower = joint.getJointLimitLower();

         if (Double.isInfinite(qUpper) || Double.isInfinite(qLower))
            continue;

         double jointRoM = qUpper - qLower;

         double qUpperCutoff0 = qUpper - jointRoM * romFraction0;
         double qUpperCutoff1 = qUpper - jointRoM * romFraction1;

         double qLowerCutoff0 = qLower + jointRoM * romFraction0;
         double qLowerCutoff1 = qLower + jointRoM * romFraction1;

         if (q > qUpperCutoff0 && qdNullspace > 0.0)
         {
            jointLimitAlpha = Math.min(jointLimitAlpha, EuclidCoreTools.clamp((qUpperCutoff1 - q) / (qUpperCutoff1 - qUpperCutoff0), 0.0, 1.0));
         }
         else if (q < qLowerCutoff0 && qdNullspace < 0.0)
         {
            jointLimitAlpha = Math.min(jointLimitAlpha, EuclidCoreTools.clamp((qLowerCutoff1 - q) / (qLowerCutoff1 - qLowerCutoff0), 0.0, 1.0));
         }
      }

      return jointLimitAlpha;
   }

   public FrameVector3DReadOnly getOptimalContactPointAdjustment(RobotSide robotSide)
   {
      return contactPointOptimalAdjustments.get(robotSide);
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
      return new DMatrixRMaj(0);
   }

   public ContactNullspaceCalculator getNullspaceCalculator()
   {
      return nullspaceCalculator;
   }

   public DMatrixRMaj getStabilityBoundaryGradient()
   {
      return stabilityBoundaryGradient;
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
