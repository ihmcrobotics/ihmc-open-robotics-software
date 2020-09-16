package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class CoPTrajectoryParameters
{
   private final double minimumDistanceInsidePolygon = 0.01;

   private final double stepLengthToPutExitCoPOnToes = 0.2;
   private final double stepHeightToPutExitCoPOnToesSteppingDown = -0.1;
   private final double stepLengthToPutExitCoPOnToesSteppingDown = 0.1;

   private final boolean planWithExitCMPOnToes = false;
   private final boolean planWithExitCMPOnToesWhenSteppingDown = true;

   private final double entryCMPMinX = -0.4;
   private final double entryCMPMaxX = 0.2;

   private final double ballCMPMinX = 0.0;
   private final double ballCMPMaxX = 0.055;

   private final double exitCMPMinX = 0.0;
   private final double exitCMPMaxX = 0.08;

   private final Vector2DReadOnly entryCMPOffset = new Vector2D(0.0, -0.005);
   private final Vector2DReadOnly ballCMPOffset = new Vector2D(0.0, 0.01);
   private final Vector2DReadOnly exitCMPOffset = new Vector2D(0.0, 0.025);

   private final double entryCMPLengthOffsetFactor = 1.0 / 3.0;
   private final double ballCMPLengthOffsetFactor = 1.0 / 8.0;
   private final double exitCMPLengthOffsetFactor = 1.0 / 3.0;

   private final PlanForToeOffCalculator planForToeOffCalculator = new PlanForToeOffCalculator()
   {
      @Override
      public boolean shouldPutCMPOnToes(double stepLength, double stepHeight)
      {
         if (getPlanWithExitCMPOnToes() && MathTools.isGreaterThanWithPrecision(stepLength, getStepLengthToPutExitCoPOnToes(), Epsilons.ONE_HUNDREDTH))
            return true;
         if (getPlanWithExitCMPOnToesWhenSteppingDown() && MathTools.isLessThanWithPrecision(stepHeight,
                                                                                             getStepHeightToPutExitCoPOnToesSteppingDown(),
                                                                                             Epsilons.ONE_HUNDREDTH) && MathTools.isGreaterThanWithPrecision(
               stepLength,
               getStepLengthToPutExitCoPOnToesSteppingDown(),
               Epsilons.ONE_HUNDREDTH))
            return true;

         return false;
      }
   };

   public double getMinimumDistanceInsidePolygon()
   {
      return minimumDistanceInsidePolygon;
   }

   public boolean getPlanWithExitCMPOnToes()
   {
      return planWithExitCMPOnToes;
   }

   public boolean getPlanWithExitCMPOnToesWhenSteppingDown()
   {
      return planWithExitCMPOnToesWhenSteppingDown;
   }

   public double getStepLengthToPutExitCoPOnToes()
   {
      return stepLengthToPutExitCoPOnToes;
   }

   public double getStepLengthToPutExitCoPOnToesSteppingDown()
   {
      return stepLengthToPutExitCoPOnToesSteppingDown;
   }

   public double getStepHeightToPutExitCoPOnToesSteppingDown()
   {
      return stepHeightToPutExitCoPOnToesSteppingDown;
   }

   public double getEntryCMPMinX()
   {
      return entryCMPMinX;
   }

   public double getEntryCMPMaxX()
   {
      return exitCMPMaxX;
   }

   public double getBallCMPMinX()
   {
      return ballCMPMinX;
   }

   public double getBallCMPMaxX()
   {
      return ballCMPMaxX;
   }

   public double getExitCMPMinX()
   {
      return exitCMPMinX;
   }

   public double getExitCMPMaxX()
   {
      return exitCMPMaxX;
   }

   public Vector2DReadOnly getEntryCMPOffset()
   {
      return entryCMPOffset;
   }

   public Vector2DReadOnly getBallCMPOffset()
   {
      return ballCMPOffset;
   }

   public Vector2DReadOnly getExitCMPOffset()
   {
      return exitCMPOffset;
   }

   public double getEntryCMPLengthOffsetFactor()
   {
      return entryCMPLengthOffsetFactor;
   }

   public double getBallCMPLengthOffsetFactor()
   {
      return ballCMPLengthOffsetFactor;
   }

   public double getExitCMPLengthOffsetFactor()
   {
      return exitCMPLengthOffsetFactor;
   }

   public PlanForToeOffCalculator getPlanForToeOffCalculator()
   {
      return planForToeOffCalculator;
   }
}
