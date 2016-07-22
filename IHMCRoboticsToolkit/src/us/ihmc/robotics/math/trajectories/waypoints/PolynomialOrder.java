package us.ihmc.robotics.math.trajectories.waypoints;

import org.ejml.data.DenseMatrix64F;

public enum PolynomialOrder {
   ORDER3,
   ORDER5;
//   ORDER7;

   public int getCoefficients()
   {
      switch (this)
      {
      case ORDER3:
         return 4;
      case ORDER5:
         return 6;
//      case ORDER7:
//         return 8;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }
   }

   public void getHBlock(double t0, double t1, DenseMatrix64F hBlockToPack)
   {
      int blockSize = this.getCoefficients() - 2;
      hBlockToPack.reshape(blockSize, blockSize);

      switch (this)
      {
//      case ORDER7:
//         hBlockToPack.set(blockSize-6, blockSize-6, 1764.0/11.0 * timeDifference(11, t0, t1));
//         hBlockToPack.set(blockSize-5, blockSize-6, 126.0       * timeDifference(10, t0, t1));
//         hBlockToPack.set(blockSize-4, blockSize-6, 280.0/3.0   * timeDifference(9,  t0, t1));
//         hBlockToPack.set(blockSize-3, blockSize-6, 63.0        * timeDifference(8,  t0, t1));
//         hBlockToPack.set(blockSize-2, blockSize-6, 36.0        * timeDifference(7,  t0, t1));
//         hBlockToPack.set(blockSize-1, blockSize-6, 14.0        * timeDifference(6,  t0, t1));
//         hBlockToPack.set(blockSize-5, blockSize-5, 100.0       * timeDifference(9,  t0, t1));
//         hBlockToPack.set(blockSize-4, blockSize-5, 75.0        * timeDifference(8,  t0, t1));
//         hBlockToPack.set(blockSize-3, blockSize-5, 360.0/7.0   * timeDifference(7,  t0, t1));
//         hBlockToPack.set(blockSize-2, blockSize-5, 30.0        * timeDifference(6,  t0, t1));
//         hBlockToPack.set(blockSize-1, blockSize-5, 12.0        * timeDifference(5,  t0, t1));
      case ORDER5:
         hBlockToPack.set(blockSize-4, blockSize-4, 400.0/7.0   * timeDifference(7,  t0, t1));
         hBlockToPack.set(blockSize-3, blockSize-4, 40.0        * timeDifference(6,  t0, t1));
         hBlockToPack.set(blockSize-2, blockSize-4, 24.0        * timeDifference(5,  t0, t1));
         hBlockToPack.set(blockSize-1, blockSize-4, 10.0        * timeDifference(4,  t0, t1));
         hBlockToPack.set(blockSize-3, blockSize-3, 144.0/5.0   * timeDifference(5,  t0, t1));
         hBlockToPack.set(blockSize-2, blockSize-3, 18.0        * timeDifference(4,  t0, t1));
         hBlockToPack.set(blockSize-1, blockSize-3, 8.0         * timeDifference(3,  t0, t1));
      case ORDER3:
         hBlockToPack.set(blockSize-2, blockSize-2, 12.0        * timeDifference(3,  t0, t1));
         hBlockToPack.set(blockSize-1, blockSize-2, 6.0         * timeDifference(2,  t0, t1));
         hBlockToPack.set(blockSize-1, blockSize-1, 4.0         * timeDifference(1,  t0, t1));
         break;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }

      for (int col = 1; col < blockSize; col++)
      {
         for (int row = 0; row < col; row++)
         {
            hBlockToPack.set(row, col, hBlockToPack.get(col, row));
         }
      }
   }

   private double timeDifference(int power, double t0, double t1)
   {
      return Math.pow(t1, power) - Math.pow(t0, power);
   }

   public boolean getPositionLine(double t, DenseMatrix64F lineToPack)
   {
      lineToPack.reshape(1, getCoefficients());
      switch (this)
      {
//      case ORDER7:
//         lineToPack.set(0, getCoefficients()-8, 1.0 * Math.pow(t, 7));
//         lineToPack.set(0, getCoefficients()-7, 1.0 * Math.pow(t, 6));
      case ORDER5:
         lineToPack.set(0, getCoefficients()-6, 1.0 * Math.pow(t, 5));
         lineToPack.set(0, getCoefficients()-5, 1.0 * Math.pow(t, 4));
      case ORDER3:
         lineToPack.set(0, getCoefficients()-4, 1.0 * Math.pow(t, 3));
         lineToPack.set(0, getCoefficients()-3, 1.0 * Math.pow(t, 2));
         lineToPack.set(0, getCoefficients()-2, 1.0 * t);
         lineToPack.set(0, getCoefficients()-1, 1.0);
         break;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }

      return true;
   }

   public boolean getVelocityLine(double t, DenseMatrix64F lineToPack)
   {
      lineToPack.reshape(1, getCoefficients());
      boolean isEndCondition = false;

      switch (this)
      {
//      case ORDER7:
//         lineToPack.set(0, getCoefficients()-8, 7.0 * Math.pow(t, 6));
//         lineToPack.set(0, getCoefficients()-7, 6.0 * Math.pow(t, 5));
      case ORDER5:
         lineToPack.set(0, getCoefficients()-6, 5.0 * Math.pow(t, 4));
         lineToPack.set(0, getCoefficients()-5, 4.0 * Math.pow(t, 3));
      case ORDER3:
         lineToPack.set(0, getCoefficients()-4, 3.0 * Math.pow(t, 2));
         lineToPack.set(0, getCoefficients()-3, 2.0 * t);
         lineToPack.set(0, getCoefficients()-2, 1.0);
         lineToPack.set(0, getCoefficients()-1, 0.0);
         isEndCondition = true;
         break;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }

      return isEndCondition;
   }

   public boolean getAccelerationLine(double t, DenseMatrix64F lineToPack)
   {
      lineToPack.reshape(1, getCoefficients());
      boolean isEndCondition = false;

      switch (this)
      {
//      case ORDER7:
//         lineToPack.set(0, getCoefficients()-8, 7.0 * 6.0 * Math.pow(t, 5));
//         lineToPack.set(0, getCoefficients()-7, 6.0 * 5.0 * Math.pow(t, 4));
      case ORDER5:
         lineToPack.set(0, getCoefficients()-6, 5.0 * 4.0 * Math.pow(t, 3));
         lineToPack.set(0, getCoefficients()-5, 4.0 * 3.0 * Math.pow(t, 2));
         isEndCondition = true;
      case ORDER3:
         lineToPack.set(0, getCoefficients()-4, 3.0 * 2.0 * t);
         lineToPack.set(0, getCoefficients()-3, 2.0 * 1.0);
         lineToPack.set(0, getCoefficients()-2, 1.0 * 0.0);
         lineToPack.set(0, getCoefficients()-1, 0.0);
         break;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }

      return isEndCondition;
   }

   public boolean getJerkLine(double t, DenseMatrix64F lineToPack)
   {
      lineToPack.reshape(1, getCoefficients());
      boolean isEndCondition = false;

      switch (this)
      {
//      case ORDER7:
//         lineToPack.set(0, getCoefficients()-8, 7.0 * 6.0 * 5.0 * Math.pow(t, 4));
//         lineToPack.set(0, getCoefficients()-7, 6.0 * 5.0 * 4.0 * Math.pow(t, 3));
//         isEndCondition = true;
      case ORDER5:
         lineToPack.set(0, getCoefficients()-6, 5.0 * 4.0 * 3.0 * Math.pow(t, 2));
         lineToPack.set(0, getCoefficients()-5, 4.0 * 3.0 * 2.0 * t);
      case ORDER3:
         lineToPack.set(0, getCoefficients()-4, 3.0 * 2.0 * 1.0);
         lineToPack.set(0, getCoefficients()-3, 2.0 * 1.0 * 0.0);
         lineToPack.set(0, getCoefficients()-2, 1.0 * 0.0);
         lineToPack.set(0, getCoefficients()-1, 0.0);
         break;
      default:
         throw new RuntimeException("Unknown Polynomial Order");
      }

      return isEndCondition;
   }
}