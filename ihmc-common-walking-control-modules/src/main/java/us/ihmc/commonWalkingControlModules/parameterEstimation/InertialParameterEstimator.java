package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.*;
import us.ihmc.parameterEstimation.inertial.InertialParameterResidualAndJacobian;
import us.ihmc.parameterEstimation.inertial.LeastSquaresL2PriorInertialParameterCalculator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.AlphaFilteredElementwiseMatrix;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class InertialParameterEstimator implements SCS2YoGraphicHolder
{
   private final YoBoolean enableEstimator;

   private final int windowSize;

   private final boolean stripOutliers;
   private final int numberOfOutliers;

   private final FullHumanoidRobotModel actualRobotModel;
   private final FullHumanoidRobotModel estimateRobotModel;
   private final JointTorqueRegressorCalculator jointTorqueRegressorCalculator;

   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final DMatrixRMaj knownParameters;
   private final DMatrixRMaj estimateParameters;

   private final InertialParameterResidualAndJacobian residualAndJacobian;
   private final DMatrixRMaj residual;
   private final YoDouble residualNorm;

   private final DMatrixRMaj wholeSystemTorques;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;
   private final SideDependentList<Wrench> contactWrenches;

   private int counter = 0;  // Variable that triggers parameter solving when we have enough data, that is, when the buffers are full

   private final StackedMeasurementHandler stackedMeasurementHandler;
   private final LeastSquaresL2PriorInertialParameterCalculator calculator;

   private final DMatrixRMaj residualNorms;
   private final SortableIndexedDoubleArray residualNormsToSort;
   private final double[] outlierNorms;
   private final int[] outlierIndices;
   private final YoDouble largestOutlierNorm;
   private final YoDouble smallestOutlierNorm;

   private final AlphaFilteredElementwiseMatrix ghostEstimate;
   private final YoMatrix filteredGhostEstimate;

   private final List<? extends JointBasics> actualModelJoints;
   private final List<? extends JointBasics> estimateModelJoints;

   public InertialParameterEstimator(HighLevelHumanoidControllerToolbox toolbox, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      enableEstimator = new YoBoolean("enableEstimator", registry);
      enableEstimator.set(false);

      NadiaInertialEstimatorParameters inertialEstimatorParameters = new NadiaInertialEstimatorParameters();
      windowSize = inertialEstimatorParameters.getWindowSize();
      stripOutliers = inertialEstimatorParameters.stripOutliers();
      numberOfOutliers = inertialEstimatorParameters.getNumberOfOutliers();
      if (numberOfOutliers > windowSize)
         throw new IllegalArgumentException("Number of outliers (" + numberOfOutliers + ") is larger than the estimator window size (" + windowSize + ")");

      int numberOfMeasurements;
      if (stripOutliers)
         numberOfMeasurements = windowSize - numberOfOutliers;
      else
         numberOfMeasurements = windowSize;

      actualRobotModel = toolbox.getFullRobotModel();
      actualModelJoints = actualRobotModel.getRootJoint().subtreeList();

      RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, true);
      estimateModelJoints = estimateRobotModel.getRootJoint().subtreeList();

      jointTorqueRegressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      jointTorqueRegressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      Map<RigidBodyReadOnly, ArrayList<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>> parametersToEstimateMap;
      parametersToEstimateMap = inertialEstimatorParameters.getParameterMapForMassFromTorsoDown(estimateRobotModel);

      int residualSize = actualRobotModel.getRootJoint().getDegreesOfFreedom() + actualRobotModel.getOneDoFJoints().length;
      RegressorHandler regressorHandler = new RegressorHandler(parametersToEstimateMap);
      int knownParameterSize = regressorHandler.getKnownParameterSize();
      int estimateParameterSize = regressorHandler.getEstimateParameterSize();

      knownParameters = new DMatrixRMaj(knownParameterSize, 1);
      estimateParameters = new DMatrixRMaj(estimateParameterSize, 1);
      fillKnownParameterVector(estimateRobotModel, parametersToEstimateMap, knownParameters, estimateParameters);

      residualAndJacobian = new InertialParameterResidualAndJacobian(estimateRobotModel, regressorHandler, knownParameters, registry);
      residual = new DMatrixRMaj(residualAndJacobian.getResidualSize(), 1);
      residualNorm = new YoDouble("residualNorm", registry);

      wholeSystemTorques = new DMatrixRMaj(residualAndJacobian.getResidualSize(), 1);

      this.footSwitches = toolbox.getFootSwitches();
      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names. The
      // estimate robot model does not
      for (RobotSide side : RobotSide.values)
      {
         legJoints.set(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.set(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.set(side, new DMatrixRMaj(6, residualAndJacobian.getResidualSize()));
      }

      contactWrenches = new SideDependentList<>((Wrench) footSwitches.get(RobotSide.LEFT).getMeasuredWrench(),
                                                (Wrench) footSwitches.get(RobotSide.RIGHT).getMeasuredWrench());

      stackedMeasurementHandler = new StackedMeasurementHandler(regressorHandler, residualSize, windowSize, numberOfMeasurements);

      calculator = new LeastSquaresL2PriorInertialParameterCalculator(residualSize, numberOfMeasurements, regressorHandler, registry);
      calculator.setKnownParameters(knownParameters);
      calculator.setEstimatePrior(estimateParameters);
      // Construct ridge matrix with different values for different links
      DMatrixRMaj ridgeMatrix = CommonOps_DDRM.identity(estimateParameterSize);
      int ridgeCounter = 0;
      for (RigidBodyReadOnly body : estimateRobotModel.getRootBody().subtreeArray())
      {
         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : parametersToEstimateMap.get(body))
         {
            ridgeMatrix.set(ridgeCounter, ridgeCounter, inertialEstimatorParameters.getRidgeParameter(body, option));
            ridgeCounter += 1;
         }
      }
      calculator.setRidgeMatrix(ridgeMatrix);

      residualNorms = new DMatrixRMaj(windowSize, 1);
      residualNormsToSort = new SortableIndexedDoubleArray(new double[windowSize]);
      outlierNorms = new double[numberOfOutliers];
      outlierIndices = new int[numberOfOutliers];
      largestOutlierNorm = new YoDouble("largestOutlierNorm", registry);
      smallestOutlierNorm = new YoDouble("smallestOutlierNorm", registry);

      ghostEstimate = new AlphaFilteredElementwiseMatrix("ghostEstimate", estimateParameterSize, 1, inertialEstimatorParameters.getEstimateFilteringAlpha(), registry);
      filteredGhostEstimate = new YoMatrix("filteredGhostEstimate", estimateParameterSize, 1, registry);

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(actualRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(actualRobotModel.getRootBody(), yoInertiaEllipsoids);
   }

   private void fillKnownParameterVector(FullHumanoidRobotModel model, Map<RigidBodyReadOnly, ArrayList<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>> parametersToEstimateByBody,
                                         DMatrixRMaj knownParametersToPack, DMatrixRMaj estimateParametersToPack)
   {
      // The indices below keep track of where we are in both the known and to estimate parameter vector
      int knownIndexCount = 0;
      int estimateIndexCount = 0;
      for (RigidBodyReadOnly body : model.getRootBody().subtreeArray())
      {
         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values)
            if (!parametersToEstimateByBody.get(body).contains(option))
               knownIndexCount = packParameters(knownParametersToPack, knownIndexCount, body, option);
            else
               estimateIndexCount = packParameters(estimateParametersToPack, estimateIndexCount, body, option);
      }
   }

   private int packParameters(DMatrixRMaj parametersToPack, int indexCount, RigidBodyReadOnly body, JointTorqueRegressorCalculator.SpatialInertiaBasisOption spatialInertiaBasisOption)
   {
      switch (spatialInertiaBasisOption)
      {
         case M -> parametersToPack.set(indexCount, 0, body.getInertia().getMass());
         case MCOM_X -> parametersToPack.set(indexCount, 0, body.getInertia().getCenterOfMassOffset().getX());
         case MCOM_Y -> parametersToPack.set(indexCount, 0, body.getInertia().getCenterOfMassOffset().getY());
         case MCOM_Z -> parametersToPack.set(indexCount, 0, body.getInertia().getCenterOfMassOffset().getZ());
         case I_XX -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM00());
         case I_XY -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM01());
         case I_XZ -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM02());
         case I_YY -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM11());
         case I_YZ -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM12());
         case I_ZZ -> parametersToPack.set(indexCount, 0, body.getInertia().getMomentOfInertia().getM22());
      }
      indexCount += 1;
      return indexCount;
   }

   public void update()
   {
      if (enableEstimator.getBooleanValue())
      {
         updateEstimatedModelJointState();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();
         updateInertiaEllipsoidGraphics();

         jointTorqueRegressorCalculator.compute();

         residualAndJacobian.setRegressor(jointTorqueRegressorCalculator.getJointTorqueRegressorMatrix());
         residualAndJacobian.setContactJacobians(fullContactJacobians);
         residualAndJacobian.setContactWrenches(contactWrenches);
         residualAndJacobian.setGeneralizedSystemTorques(wholeSystemTorques);
         residualAndJacobian.update();
         residualAndJacobian.calculateResidual(estimateParameters, residual);
         residualNorm.set(NormOps_DDRM.fastNormP2(residual));

         // Slide the previous residual norms upward and add the residual norm just calculated
         MatrixMissingTools.slideBottomNRowsToTop(residualNorms, windowSize - 1);
         residualNorms.set(windowSize - 1, residualNorm.getDoubleValue());

         // Also put the residual norms into a sortable array that we'll use to get the indices of measurements to discard as outliers
         for (int i = 0; i < windowSize - 1; ++i)
            residualNormsToSort.set(i, residualNorms.get(i));
         residualNormsToSort.set(windowSize - 1, residualNorm.getDoubleValue());

         // Sort the residual norms, so we can discard the largest ones
         residualNormsToSort.sort();

         for (int j = 0; j < numberOfOutliers; ++j)
         {
            outlierIndices[j] = residualNormsToSort.getIndex(j);
            outlierNorms[j] = residualNormsToSort.get(j);
         }
         largestOutlierNorm.set(outlierNorms[0]);
         smallestOutlierNorm.set(outlierNorms[numberOfOutliers - 1]);

         stackedMeasurementHandler.updateStackedMeasurements(residualAndJacobian.getGeneralizedSystemTorques(),
                                                             residualAndJacobian.getGeneralizedContactTorques(),
                                                             residualAndJacobian.getKnownRegressor(),
                                                             residualAndJacobian.getEstimateRegressor());

         if (stripOutliers)
         {
            stackedMeasurementHandler.stripOutliers(outlierIndices);

            calculator.updateStackedQuantities(stackedMeasurementHandler.getOutlierStrippedGeneralizedSystemTorques(),
                                               stackedMeasurementHandler.getOutlierStrippedGeneralizedContactTorques(),
                                               stackedMeasurementHandler.getOutlierStrippedKnownRegressor(),
                                               stackedMeasurementHandler.getOutlierStrippedEstimateRegressor());
         }
         else
            calculator.updateStackedQuantities(stackedMeasurementHandler.getStackedGeneralizedSystemTorques(),
                                               stackedMeasurementHandler.getStackedGeneralizedContactTorques(),
                                               stackedMeasurementHandler.getStackedKnownRegressor(),
                                               stackedMeasurementHandler.getStackedEstimateRegressor());

         if (counter > windowSize)
         {
            calculator.solve();
            ghostEstimate.setAndSolve(calculator.getEstimate());
            filteredGhostEstimate.set(ghostEstimate.getFilteredMatrix());

            estimateParameters.set(calculator.getEstimate());
         }
         counter += 1;
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side: RobotSide.values)
         contactWrenches.get(side).set(footSwitches.get(side).getMeasuredWrench());
   }

   private void updateEstimatedModelJointState()
   {
      for (JointStateType type : JointStateType.values())
         MultiBodySystemTools.copyJointsState(actualModelJoints, estimateModelJoints, type);
      estimateRobotModel.getRootJoint().updateFramesRecursively();
   }

   private void updateWholeSystemTorques()
   {
      actualRobotModel.getRootJoint().getJointTau(0, wholeSystemTorques);
      for (OneDoFJointReadOnly joint : actualRobotModel.getOneDoFJoints())
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         joint.getJointTau(jointIndex, wholeSystemTorques);
      }
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   //TODO: adjust this to be smarter and only update the ellipsoids that are being estimated
   private void updateInertiaEllipsoidGraphics()
   {
      RigidBodyBasics[] estimateBodies = estimateRobotModel.getRootBody().subtreeArray();
      RigidBodyBasics[] actualBodies = actualRobotModel.getRootBody().subtreeArray();
      for (int i = 0; i < estimateBodies.length; i++)
      {
         RigidBodyBasics estimateBody = estimateBodies[i];
         RigidBodyBasics actualBody = actualBodies[i];
         if(estimateBody.isRootBody())
            continue;

         //FIXME (CD): estimateRobotModel is not updated after the first tick, we need to grab the mass another way
         double estimateBodyMass = estimateBody.getInertia().getMass();
         double actualBodyMass = actualBody.getInertia().getMass();
         double massRatio = (estimateBodyMass - actualBodyMass) / actualBodyMass;
         // This assumes that the highest mass we will see is 2x the actual mass, we can normalize to something else if we want
         double massNormalizedScale = massRatio/2.0 + 0.5;
         // We enforce that the scale is in the range [0, 1]
         massNormalizedScale = Math.max(0.0, Math.min(1.0, massNormalizedScale));

         InertiaVisualizationTools.updateEllipsoid(yoInertiaEllipsoids.get(i), massNormalizedScale);
      }
   }

   /**
    * Class that holds 1) an array of doubles, and 2) an array of indices associated to the array of doubles. The idea is, when we sort our array, we would
    * like to keep track of what entries end up where, in case we need to index another object with that sorted order.
    */
   private static class SortableIndexedDoubleArray
   {
      private final double[] arrayToSort;
      private final int[] indices;

      private boolean isSorted = false;

      public SortableIndexedDoubleArray(double[] input)
      {
         arrayToSort = input;
         indices = new int[input.length];
         resetIndices();
      }

      /** Each time we set either an element of the array, or the whole array, we reset the associated indices, so they are an ascending sequence of integers. */
      private void resetIndices()
      {
         if (isSorted)
         {
            for (int i =  0; i < indices.length; ++i)
               indices[i] = i;
            isSorted = false;
         }
      }

      private void swapValueAndIndex(int index1, int index2)
      {
         double temporaryValue = arrayToSort[index1];
         arrayToSort[index1] = arrayToSort[index2];
         arrayToSort[index2] = temporaryValue;

         int temporaryIndex = indices[index1];
         indices[index1] = indices[index2];
         indices[index2] = temporaryIndex;
      }

      public void sort()
      {
         boolean isOrdered = false;

         while (!isOrdered)
         {
            isOrdered = true;
            for (int i = 0; i < arrayToSort.length - 1; i++)
            {
               double a = arrayToSort[i];
               double b = arrayToSort[i + 1];

               if(a < b)
               {
                  isOrdered = false;
                  swapValueAndIndex(i, i + 1);
               }
            }
         }
         isSorted = true;
      }

      public void set(double[] input)
      {
         if (arrayToSort.length != input.length)
            throw new IllegalArgumentException("Wrong input size, expected " + arrayToSort.length + " but received " + input.length);

         System.arraycopy(input, 0, arrayToSort, 0, input.length);
         resetIndices();
      }

      public void set(int index, double value)
      {
         arrayToSort[index] = value;
         resetIndices();
      }

      public double[] get()
      {
         return arrayToSort;
      }

      public double get(int index)
      {
         return arrayToSort[index];
      }

      /** This isn't much use if you haven't called sort(), it will just return an ordered list of integers */
      public int[] getIndices()
      {
         return indices;
      }

      /** This isn't much use if you haven't called sort(), it will just return the same index back */
      public int getIndex(int index)
      {
         return indices[index];
      }

      public boolean isSorted()
      {
         return isSorted;
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }
}
