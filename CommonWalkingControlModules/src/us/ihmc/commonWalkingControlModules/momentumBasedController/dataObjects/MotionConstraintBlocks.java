package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;


public class MotionConstraintBlocks
{
   private final int motionConstraintIndex;
   private final DenseMatrix64F jFullBlock;
   private final DenseMatrix64F jBlockCompact;
   private final DenseMatrix64F pBlock;
   private final double weightBlock;

   public MotionConstraintBlocks(int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, double weightBlock)
   {
      this.motionConstraintIndex = motionConstraintIndex;
      this.jFullBlock = new DenseMatrix64F(jFullBlock);
      this.jBlockCompact = new DenseMatrix64F(jBlockCompact);
      this.pBlock = new DenseMatrix64F(pBlock);
      this.weightBlock = weightBlock;
   }

   public int getMotionConstraintIndex()
   {
      return motionConstraintIndex;
   }

   public DenseMatrix64F getjFullBlock()
   {
      return jFullBlock;
   }

   public DenseMatrix64F getjBlockCompact()
   {
      return jBlockCompact;
   }

   public DenseMatrix64F getpBlock()
   {
      return pBlock;
   }

   public double getWeightBlock()
   {
      return weightBlock;
   }
}
