package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import org.apache.commons.lang.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MotionConstraintBlocks;
import us.ihmc.commonWalkingControlModules.momentumBasedController.gui.DesiredJointAccelerationJPanel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredMomentumModuleCommandListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleSolverListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionConstraintListener;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

public class AllMomentumModuleListener implements MotionConstraintListener, DesiredMomentumModuleCommandListener, MomentumControlModuleSolverListener
{
   private final HashMap<Object, MotionConstraintBlocks> motionConstraintBlocksMap = new HashMap<Object, MotionConstraintBlocks>();

   private final MomentumModuleDataObject momentumModuleDataObject = new MomentumModuleDataObject();

   private final ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = new ArrayList<DesiredRateOfChangeOfMomentumCommand>();

   private final ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = new ArrayList<DesiredJointAccelerationCommand>();
   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = new ArrayList<DesiredSpatialAccelerationCommand>();
   private final ArrayList<DesiredPointAccelerationCommand> desiredPointAccelerationCommands = new ArrayList<DesiredPointAccelerationCommand>();


   // Solution
   private DenseMatrix64F jointAccelerationsSolution = new DenseMatrix64F(1, 1);
   private InverseDynamicsJoint[] jointsToOptimizeFor;
   
   private double optimizationValue;

   // Analysis
   private final ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredJointAccelerationCommandAndMotionConstraint>();
   private final ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint>();
   private final ArrayList<DesiredPointAccelerationCommandAndMotionConstraint> desiredPointAccelerationCommandAndMotionConstraints =
      new ArrayList<DesiredPointAccelerationCommandAndMotionConstraint>();

   private final NumberFormat numberFormat;
   
   public AllMomentumModuleListener()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(5);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public void jointAccelerationMotionConstraintWasAdded(DesiredJointAccelerationCommand desiredJointAccelerationCommand, int motionConstraintIndex,
           DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredJointAccelerationCommand, motionConstraintBlocks);

//      System.out.println("---Joint Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//                         + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//      System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());

   }

   public void spatialAccelerationMotionConstraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex,
           DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredSpatialAccelerationCommand, motionConstraintBlocks);

//      System.out.println("---Spatial Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//                         + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//      System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
   }

   public void pointAccelerationMotionConstraintWasAdded(DesiredPointAccelerationCommand desiredPointAccelerationCommand, int motionConstraintIndex,
           DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      double weight = extractWeight(weightBlock);
      
      MotionConstraintBlocks motionConstraintBlocks = new MotionConstraintBlocks(motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weight);
      motionConstraintBlocksMap.put(desiredPointAccelerationCommand, motionConstraintBlocks);

//      System.out.println("---Point Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock
//                         + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//      System.out.println("---motionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());

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

      desiredRateOfChangeOfMomentumCommands.clear();
      desiredJointAccelerationCommands.clear();
      desiredSpatialAccelerationCommands.clear();
      desiredPointAccelerationCommands.clear();

      desiredJointAccelerationCommandAndMotionConstraints.clear();
      desiredSpatialAccelerationCommandAndMotionConstraints.clear();
      desiredPointAccelerationCommandAndMotionConstraints.clear();

   }

   public void setCentroidalMomentumMatrix(DenseMatrix64F a)
   {
      // TODO Auto-generated method stub

   }

   public void setMomentumDotEquationRightHandSide(DenseMatrix64F b)
   {
      // TODO Auto-generated method stub

   }

   public void setPrimaryMotionConstraintJMatrix(DenseMatrix64F jPrimary)
   {
      // TODO Auto-generated method stub

   }

   public void setPrimaryMotionConstraintPVector(DenseMatrix64F pPrimary)
   {
      // TODO Auto-generated method stub

   }

   public void setPrimaryMotionConstraintCheck(DenseMatrix64F checkCopy)
   {
      // TODO Auto-generated method stub

   }

   public void setCheckJQEqualsZeroAfterSetConstraint(DenseMatrix64F denseMatrix64F)
   {
      // TODO Auto-generated method stub

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
      this.jointAccelerationsSolution.setReshape(jointAccelerationsSolution);
      System.out.println("\n\n***********\njointAccelerationsSolution = " + jointAccelerationsSolution);

   }

   public void setOptimizationValue(double optimizationValue)
   {
      this.optimizationValue = optimizationValue;
   }

   public void reviewSolution()
   {
      
      System.out.println("\nmotionConstraintBlocksMap.size() = " + motionConstraintBlocksMap.size());
      System.out.println("desiredJointAccelerationCommands.size() = " + desiredJointAccelerationCommands.size());
      System.out.println("desiredSpatialAccelerationCommands.size() = " + desiredSpatialAccelerationCommands.size());
      System.out.println("desiredPointAccelerationCommands.size() = " + desiredPointAccelerationCommands.size() + "\n");
      
      for (DesiredJointAccelerationCommand desiredJointAccelerationCommand : desiredJointAccelerationCommands)
      {
         MotionConstraintBlocks motionConstraintBlocks = motionConstraintBlocksMap.get(desiredJointAccelerationCommand);

         if (motionConstraintBlocks != null)
         {
            DesiredJointAccelerationCommandAndMotionConstraint commandAndConstraint =
                  new DesiredJointAccelerationCommandAndMotionConstraint(desiredJointAccelerationCommand, motionConstraintBlocks);
            commandAndConstraint.computeAchievedJointAcceleration(jointAccelerationsSolution);

            String desiredString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, desiredJointAccelerationCommand.getDesiredAcceleration());
            String achievedString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, commandAndConstraint.getAchievedJointAcceleration());
            System.out.println("+++desiredJointAcceleration + " + desiredJointAccelerationCommand.getJoint().getName() + " " + desiredString + ", " + achievedString);

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

            SpatialAccelerationVector desiredSpatialAcceleration = desiredSpatialAccelerationCommand.getTaskspaceConstraintData().getSpatialAcceleration();
            Vector3d angularPart = desiredSpatialAcceleration.getAngularPartCopy();
            Vector3d linearPart = desiredSpatialAcceleration.getLinearPartCopy();
            
            String desiredString = angularPart.toString() + " " + linearPart.toString();
            String achievedString = DesiredJointAccelerationJPanel.toPrettyString(numberFormat, commandAndConstraint.getAchievedSpatialAcceleration());
            System.out.println("+++desiredSpatialAcceleration + " + "" + " " + desiredString + ", " + achievedString);
            
            desiredSpatialAccelerationCommandAndMotionConstraints.add(commandAndConstraint);
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

            System.out.println("+++desiredPointAcceleration = " + desiredPointAccelerationCommand.getDesiredAcceleration());
            System.out.println("achievedPointAcceleration = " + commandAndConstraint.getAchievedPointAcceleration());

            desiredPointAccelerationCommandAndMotionConstraints.add(commandAndConstraint);
         }
      }
   }


   public ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> getDesiredJointAccelerationCommandAndMotionConstraints()
   {
      return desiredJointAccelerationCommandAndMotionConstraints;
   }
   
   public ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> getDesiredSpatialAccelerationCommandAndMotionConstraints()
   {
      return desiredSpatialAccelerationCommandAndMotionConstraints;
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

}
