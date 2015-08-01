package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MotionConstraintBlocks;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredMomentumModuleCommandListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionConstraintListener;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class AllMomentumModuleListener implements MotionConstraintListener, DesiredMomentumModuleCommandListener, MomentumControlModuleSolverListener
{   
   private final HashMap<Object, MotionConstraintBlocks> motionConstraintBlocksMap = new HashMap<Object, MotionConstraintBlocks>();
   private final HashMap<Object, MotionConstraintBlocks> nullspaceMotionConstraintBlocks = new HashMap<Object, MotionConstraintBlocks>();
   
   private final MomentumModuleDataObject momentumModuleDataObject = new MomentumModuleDataObject();

   private final ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = new ArrayList<DesiredRateOfChangeOfMomentumCommand>();

   private final ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = new ArrayList<DesiredJointAccelerationCommand>();
   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = new ArrayList<DesiredSpatialAccelerationCommand>();
   private final ArrayList<DesiredPointAccelerationCommand> desiredPointAccelerationCommands = new ArrayList<DesiredPointAccelerationCommand>();

   // J and P matrices:
   private final DenseMatrix64F jPrimaryMotionConstraint = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F pPrimaryMotionConstraint = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F primaryMotionConstraintCheck = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F checkJQEqualsZero = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F pAchievedPrimaryConstraints = new DenseMatrix64F(1, 1);
   
   // Solution
   private DenseMatrix64F jointAccelerationsSolution = new DenseMatrix64F(1, 1);
   private InverseDynamicsJoint[] jointsToOptimizeFor;

   // Analysis
   private final ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredJointAccelerationCommandAndMotionConstraint>();
   private final ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint>();
   private final ArrayList<DesiredPointAccelerationCommandAndMotionConstraint> desiredPointAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredPointAccelerationCommandAndMotionConstraint>();
   
   private final ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndNullspaceMotionConstraints =
         new ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BooleanYoVariable printMotionConstraints = new BooleanYoVariable("printMotionConstraints", registry);
   
   private final DoubleYoVariable checkJQEqualsZeroMax = new DoubleYoVariable("checkJQEqualsZeroMax", registry);
   private final DoubleYoVariable minimumSingularValue = new DoubleYoVariable("minimumSingularValue", registry);
   private final DoubleYoVariable jacobianMatrixSmallestSingularValue = new DoubleYoVariable("jacobianMatrixSmallestSingularValue", registry);
   
   private final IntegerYoVariable numberJointAccelerationCommands = new IntegerYoVariable("numberJointAccelerationCommands", registry);
   private final IntegerYoVariable numberSpatialAccelerationCommands = new IntegerYoVariable("numberSpatialAccelerationCommands", registry);
   private final IntegerYoVariable numberPointAccelerationCommands = new IntegerYoVariable("numberPointAccelerationCommands", registry);
   
   
   private final IntegerYoVariable numberMotionConstraints = new IntegerYoVariable("numberMotionConstraints", registry);
   private final IntegerYoVariable numberNullspaceConstraints = new IntegerYoVariable("numberNullspaceConstraints", registry);
   
   private final IntegerYoVariable numberRowsInPrimaryMotionConstraints = new IntegerYoVariable("numberRowsInPrimaryMotionConstraints", registry);
   
   private final YoFrameVector desiredLinearMomentumRate = new YoFrameVector("desiredLinearMomentumRate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector achievedLinearMomentumRate = new YoFrameVector("achievedLinearMomentumRate", null, registry);

   private final YoFrameVector desiredAngularMomentumRate = new YoFrameVector("desiredAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector achievedAngularMomentumRate = new YoFrameVector("achievedAngularMomentumRate", null, registry);

   
   private final DoubleYoVariable centroidalMomentumSmallestSingularValue = new DoubleYoVariable("centroidalMomentumSmallestSingularValue", registry);
   private final DoubleYoVariable optimizationValue = new DoubleYoVariable("optimizationValue", registry);

   
   private final NumberFormat numberFormat;
   
   public AllMomentumModuleListener(YoVariableRegistry parentRegistry)
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(5);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
      
      parentRegistry.addChild(registry);
   }
   
   public void jointAccelerationMotionConstraintWasAdded(DesiredJointAccelerationCommand desiredJointAccelerationCommand, int motionConstraintIndex,
           DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredJointAccelerationCommand, motionConstraintBlocks);

      if (printMotionConstraints.getBooleanValue())
      {
         String frameName = desiredJointAccelerationCommand.getJoint().getName();
         
         System.out.println();
         MatrixTools.printJavaForConstruction(frameName + "JointAccelerationJMatrix", jFullBlock);
         MatrixTools.printJavaForConstruction(frameName + "JointAccelerationPVector", pBlock);
         
//         System.out.println("---Joint Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//               + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//         System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
      }
   }
   
  

   public void spatialAccelerationMotionConstraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex,
         DenseMatrix64F jacobianMatrix, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredSpatialAccelerationCommand, motionConstraintBlocks);

      SingularValueDecomposition<DenseMatrix64F> svd = computeSVD(jacobianMatrix);      
      double[] singularValues = svd.getSingularValues();
      double smallestSingularValue = computeSmallestSingularValue(singularValues);
      
      DenseMatrix64F matrixW = new DenseMatrix64F(jacobianMatrix.getNumRows(), jacobianMatrix.getNumCols());
      svd.getW(matrixW);
      
//      System.out.println("jacobianMatrixSmallestSingularValue = " + smallestSingularValue);
//      System.out.println("jacobianMatrix = " + jacobianMatrix);
      if (smallestSingularValue < jacobianMatrixSmallestSingularValue.getDoubleValue())
      {
         jacobianMatrixSmallestSingularValue.set(smallestSingularValue);
      }
      
      if (printMotionConstraints.getBooleanValue())
      {
         String frameName = desiredSpatialAccelerationCommand.getTaskspaceConstraintData().getSpatialAcceleration().getBodyFrame().getName();
        
         System.out.println();
         MatrixTools.printJavaForConstruction(frameName + "SpatialAccelerationJMatrix", jFullBlock);
         MatrixTools.printJavaForConstruction(frameName + "SpatialAccelerationPVector", pBlock);
         
         
//         System.out.println("---Spatial Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//               + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//         System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
      }
   }
   
   public void nullSpaceMultiplierForSpatialAccelerationMotionContraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand,
         int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      nullspaceMotionConstraintBlocks.put(desiredSpatialAccelerationCommand, motionConstraintBlocks);

      //TODO: Figure out how to record this since it will have the same desiredSpatialAccelerationCommand as its associated desiredSpatialAccelerationCommand.
      motionConstraintBlocksMap.put(desiredSpatialAccelerationCommand, motionConstraintBlocks);

      if (printMotionConstraints.getBooleanValue())
      {
         String frameName = desiredSpatialAccelerationCommand.getTaskspaceConstraintData().getSpatialAcceleration().getBodyFrame().getName();
         
         System.out.println();
         MatrixTools.printJavaForConstruction(frameName + "NullSpaceJMatrix", jFullBlock);
         MatrixTools.printJavaForConstruction(frameName + "NullSpacePVector", pBlock);
         
//         System.out.println("---Null Space MultiplierFor Spatial Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//               + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//         System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
      }
   }
  

   public void pointAccelerationMotionConstraintWasAdded(DesiredPointAccelerationCommand desiredPointAccelerationCommand, int motionConstraintIndex,
           DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredPointAccelerationCommand, motionConstraintBlocks);

      if (printMotionConstraints.getBooleanValue())
      {
         String frameName = desiredPointAccelerationCommand.getRootToEndEffectorJacobian().getBaseFrame().getName();

         System.out.println();
         MatrixTools.printJavaForConstruction(frameName + "PointAccelerationJMatrix", jFullBlock);
         MatrixTools.printJavaForConstruction(frameName + "PointAccelerationPVector", pBlock);
         
//         System.out.println("---Point Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//               + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//         System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
      }

   }

   private double extractWeight(MutableDouble weightBlock)
   {
      double weight;
      if (weightBlock == null) weight = Double.POSITIVE_INFINITY;
      else weight = (Double) weightBlock.getValue();
      return weight;
   }
   
   public void desiredRateOfChangeOfMomentumWasSet(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      desiredRateOfChangeOfMomentumCommands.add(desiredRateOfChangeOfMomentumCommand);

      MomentumRateOfChangeData momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      
      //TODO: Display these in GUI.
      desiredLinearMomentumRate.set(momentumMultipliers.get(0), momentumMultipliers.get(1), momentumMultipliers.get(2));

//      System.out.println("desiredRateOfChangeOfMomentum was set: " + desiredRateOfChangeOfMomentumCommand);
   }

   public void desiredJointAccelerationWasSet(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      desiredJointAccelerationCommands.add(desiredJointAccelerationCommand);

//      System.out.println("000 desiredJointAcceleration was set: " + desiredJointAccelerationCommand);
   }

   public void desiredSpatialAccelerationWasSet(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      desiredSpatialAccelerationCommands.add(desiredSpatialAccelerationCommand);

//      System.out.println("desiredSpatialAcceleration was set: " + desiredSpatialAccelerationCommand);
   }

   public void desiredPointAccelerationWasSet(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      desiredPointAccelerationCommands.add(desiredPointAccelerationCommand);

//      System.out.println("desiredPointAcceleration was set: " + desiredPointAccelerationCommand);
   }


   public void reset()
   {
      momentumModuleDataObject.reset();
      motionConstraintBlocksMap.clear();
      nullspaceMotionConstraintBlocks.clear();
      
      desiredRateOfChangeOfMomentumCommands.clear();
      desiredJointAccelerationCommands.clear();
      desiredSpatialAccelerationCommands.clear();
      desiredPointAccelerationCommands.clear();

      desiredJointAccelerationCommandAndMotionConstraints.clear();
      desiredSpatialAccelerationCommandAndMotionConstraints.clear();
      desiredSpatialAccelerationCommandAndNullspaceMotionConstraints.clear();
      desiredPointAccelerationCommandAndMotionConstraints.clear();

      jacobianMatrixSmallestSingularValue.set(Double.POSITIVE_INFINITY);
   }

   private final DenseMatrix64F centroidalMomentumAMatrixCopy = new DenseMatrix64F(0);
   private final DenseMatrix64F momentumDotEquationRightHandSideCopy = new DenseMatrix64F(0);
   private final DenseMatrix64F momentumSubspaceCopy = new DenseMatrix64F(0);
   
   public void setCentroidalMomentumMatrix(DenseMatrix64F centroidalMomentumAMatrix, DenseMatrix64F momentumDotEquationRightHandSide, DenseMatrix64F momentumSubspace)
   { 
      centroidalMomentumAMatrixCopy.set(centroidalMomentumAMatrix);
      momentumDotEquationRightHandSideCopy.set(momentumDotEquationRightHandSide);
      momentumSubspaceCopy.set(momentumSubspace);
      
      if (printMotionConstraints.getBooleanValue())
      {
         System.out.println();
         MatrixTools.printJavaForConstruction("centroidalMomentumAMatrix", centroidalMomentumAMatrixCopy);
         
         System.out.println();
         MatrixTools.printJavaForConstruction("momentumDotEquationRightHandSide", momentumDotEquationRightHandSideCopy);
         
         System.out.println();
         MatrixTools.printJavaForConstruction("momentumSubspace", momentumSubspaceCopy);
      }
      
      SingularValueDecomposition<DenseMatrix64F> svd = computeSVD(centroidalMomentumAMatrixCopy);      
      double[] singularValues = svd.getSingularValues();
      centroidalMomentumSmallestSingularValue.set(computeSmallestSingularValue(singularValues));
   }

   public void setPrimaryMotionConstraintJMatrix(DenseMatrix64F jPrimary)
   {
      jPrimaryMotionConstraint.set(jPrimary);
      
      if (printMotionConstraints.getBooleanValue())
      {
         System.out.println();
         MatrixTools.printJavaForConstruction("JPrimaryMotionConstraints", jPrimaryMotionConstraint);
      }
                 
      SingularValueDecomposition<DenseMatrix64F> svd = computeSVD(jPrimaryMotionConstraint);
      double[] singularValues = svd.getSingularValues();
      double minSingularValue = computeSmallestSingularValue(singularValues);
      minimumSingularValue.set(minSingularValue);
   }

   private static SingularValueDecomposition<DenseMatrix64F> computeSVD(DenseMatrix64F matrix)
   {
      int numRows = matrix.getNumRows();
      int numColumns = matrix.getNumCols();
      boolean needU = true;
      boolean needV = true;
      boolean compact = false;
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(numRows, numColumns, needU, needV, compact);
      svd.decompose(matrix);
      return svd;
   }
   
   private static double computeSmallestSingularValue(double[] singularValues)
   {
      double minSingularValue = Double.POSITIVE_INFINITY;
      for (int i=0; i<singularValues.length; i++)
      {
         double singularValue = singularValues[i];
//         System.out.print(singularValue + " ");
         
         if (singularValue < minSingularValue) minSingularValue = singularValue;
      }
      return minSingularValue;
   }

   public void setPrimaryMotionConstraintPVector(DenseMatrix64F pPrimary)
   {
      pPrimaryMotionConstraint.set(pPrimary);
      
      if (printMotionConstraints.getBooleanValue())
      {
         System.out.println();
         MatrixTools.printJavaForConstruction("pPrimaryMotionConstraints", pPrimaryMotionConstraint);
      }
      
//      System.out.println("!!! PrimaryMotionConstraintPMatrix = " + pPrimaryMotionConstraint);

   }

   public void setPrimaryMotionConstraintCheck(DenseMatrix64F primaryCheck)
   {
      primaryMotionConstraintCheck.set(primaryCheck);
      
//      System.out.println("!!! PrimaryMotionConstraint Check = " + primaryMotionConstraintCheck);
   }

   public void setCheckJQEqualsZeroAfterSetConstraint(DenseMatrix64F checkJQEqualsZero)
   {
      this.checkJQEqualsZero.set(checkJQEqualsZero);
      
//      System.out.println("!!! PrimaryMotionConstraint checkJQEqualsZero = " + this.checkJQEqualsZero);
   }

   public void setSecondaryMotionConstraintJMatrix(DenseMatrix64F jSecondary)
   {
      // TODO Auto-generated method stub

   }

   public void setSecondaryMotionConstraintPVector(DenseMatrix64F pSecondary)
   {
      // TODO Auto-generated method stub

   }

   public void setSecondaryMotionConstraintWeightMatrix(DenseMatrix64F weightMatrixSecondary)
   {
      // TODO Auto-generated method stub

   }

   public void setJointAccelerationSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerationsSolution)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointAccelerationsSolution.set(jointAccelerationsSolution);
//      System.out.println("\n\n***********\njointAccelerationsSolution = " + this.jointAccelerationsSolution);

   }

   public void setOptimizationValue(double optimizationValue)
   {
      this.optimizationValue.set(optimizationValue);
   }

   public void momentumModuleSolutionWasComputed(MomentumModuleSolution momentumModuleSolution)
   {
      SpatialForceVector centroidalMomentumRateSolution = momentumModuleSolution.getCentroidalMomentumRateSolution();
      
      achievedAngularMomentumRate.set(centroidalMomentumRateSolution.getAngularPartCopy());
      achievedLinearMomentumRate.set(centroidalMomentumRateSolution.getLinearPartCopy());
   }
   
   public void reviewSolution()
   {
//      System.out.println("\nmotionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
//      System.out.println("desiredJointAccelerationCommands.size() = " + desiredJointAccelerationCommands.size());
//      System.out.println("desiredSpatialAccelerationCommands.size() = " + desiredSpatialAccelerationCommands.size());
//      System.out.println("desiredPointAccelerationCommands.size() = " + desiredPointAccelerationCommands.size() + "\n");
      
      numberJointAccelerationCommands.set(desiredJointAccelerationCommands.size());
      numberSpatialAccelerationCommands.set(desiredSpatialAccelerationCommands.size());
      numberPointAccelerationCommands.set(desiredPointAccelerationCommands.size());
      
      numberMotionConstraints.set(motionConstraintBlocksMap.size());
      numberNullspaceConstraints.set(nullspaceMotionConstraintBlocks.size());

      numberRowsInPrimaryMotionConstraints.set(jPrimaryMotionConstraint.getNumRows());
      for (DesiredJointAccelerationCommand desiredJointAccelerationCommand : desiredJointAccelerationCommands)
      {
         MotionConstraintBlocks motionConstraintBlocks = motionConstraintBlocksMap.get(desiredJointAccelerationCommand);

         if (motionConstraintBlocks != null)
         {
            DesiredJointAccelerationCommandAndMotionConstraint commandAndConstraint =
                  new DesiredJointAccelerationCommandAndMotionConstraint(desiredJointAccelerationCommand, motionConstraintBlocks);
            commandAndConstraint.computeAchievedJointAcceleration(jointAccelerationsSolution);

//            String desiredString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, desiredJointAccelerationCommand.getDesiredAcceleration());
//            String achievedString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, commandAndConstraint.getAchievedJointAcceleration());
//            System.out.println("+++desiredJointAcceleration + " + desiredJointAccelerationCommand.getJoint().getName() + " " + desiredString + ", " + achievedString);

            desiredJointAccelerationCommandAndMotionConstraints.add(commandAndConstraint);
         }
      }

      for (DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand : desiredSpatialAccelerationCommands)
      {
         MotionConstraintBlocks motionConstraintBlocks = motionConstraintBlocksMap.get(desiredSpatialAccelerationCommand);

         if (motionConstraintBlocks != null)
         {
            DesiredSpatialAccelerationCommandAndMotionConstraint commandAndConstraint =
                  new DesiredSpatialAccelerationCommandAndMotionConstraint(desiredSpatialAccelerationCommand, motionConstraintBlocks);
            commandAndConstraint.computeAchievedSpatialAcceleration(jointAccelerationsSolution);

//            SpatialAccelerationVector desiredSpatialAcceleration = desiredSpatialAccelerationCommand.getTaskspaceConstraintData().getSpatialAcceleration();
//            Vector3d angularPart = desiredSpatialAcceleration.getAngularPartCopy();
//            Vector3d linearPart = desiredSpatialAcceleration.getLinearPartCopy();
            
//            String desiredString = angularPart.toString() + " " + linearPart.toString();
//            String achievedString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, commandAndConstraint.getAchievedSpatialAcceleration());
//            System.out.println("+++desiredSpatialAcceleration + " + "" + " " + desiredString + ", " + achievedString);
            
            desiredSpatialAccelerationCommandAndMotionConstraints.add(commandAndConstraint);
         }
         
         motionConstraintBlocks = nullspaceMotionConstraintBlocks.get(desiredSpatialAccelerationCommand);

         if (motionConstraintBlocks != null)
         {
            DesiredSpatialAccelerationCommandAndMotionConstraint commandAndConstraint =
                  new DesiredSpatialAccelerationCommandAndMotionConstraint(desiredSpatialAccelerationCommand, motionConstraintBlocks);
            commandAndConstraint.computeAchievedSpatialAcceleration(jointAccelerationsSolution);

            desiredSpatialAccelerationCommandAndNullspaceMotionConstraints.add(commandAndConstraint);
         }
      }

      for (DesiredPointAccelerationCommand desiredPointAccelerationCommand : desiredPointAccelerationCommands)
      {
         MotionConstraintBlocks motionConstraintBlocks = motionConstraintBlocksMap.get(desiredPointAccelerationCommand);

         if (motionConstraintBlocks != null)
         {
            DesiredPointAccelerationCommandAndMotionConstraint commandAndConstraint =
                  new DesiredPointAccelerationCommandAndMotionConstraint(desiredPointAccelerationCommand, motionConstraintBlocks);
            commandAndConstraint.computeAchievedSpatialAcceleration(jointAccelerationsSolution);

//            System.out.println("+++desiredPointAcceleration = " + desiredPointAccelerationCommand.getDesiredAcceleration());
//            System.out.println("achievedPointAcceleration = " + commandAndConstraint.getAchievedPointAcceleration());

            desiredPointAccelerationCommandAndMotionConstraints.add(commandAndConstraint);
         }
      }
      
      pAchievedPrimaryConstraints.reshape(jPrimaryMotionConstraint.getNumRows(), 1);
      CommonOps.mult(jPrimaryMotionConstraint, jointAccelerationsSolution, pAchievedPrimaryConstraints);
      
//      System.out.println("!!! pPrimaryMotionConstraint = " + pPrimaryMotionConstraint);
//      System.out.println("!!! pAchievedPrimaryConstraints = " + pAchievedPrimaryConstraints);
//      System.out.println("!!!checkJQEqualsZero = " + checkJQEqualsZero);
      
      checkJQEqualsZeroMax.set(CommonOps.elementMaxAbs(checkJQEqualsZero));
   }


   public ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> getDesiredJointAccelerationCommandAndMotionConstraints()
   {
      return desiredJointAccelerationCommandAndMotionConstraints;
   }
   
   public ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> getDesiredSpatialAccelerationCommandAndMotionConstraints()
   {
      return desiredSpatialAccelerationCommandAndMotionConstraints;
   }
   
   public ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> getDesiredSpatialAccelerationCommandAndNullspaceMotionConstraints()
   {
      return desiredSpatialAccelerationCommandAndNullspaceMotionConstraints;
   }
   
   
   public ArrayList<DesiredPointAccelerationCommandAndMotionConstraint> getDesiredPointAccelerationCommandAndMotionConstraints()
   {
      return desiredPointAccelerationCommandAndMotionConstraints;
   }

   public DenseMatrix64F getJointAccelerationsSolution()
   {
      return jointAccelerationsSolution;
   }
   
   public InverseDynamicsJoint[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public DenseMatrix64F getPrimaryMotionConstraintJMatrix()
   {
      return jPrimaryMotionConstraint;
   }

   

}
