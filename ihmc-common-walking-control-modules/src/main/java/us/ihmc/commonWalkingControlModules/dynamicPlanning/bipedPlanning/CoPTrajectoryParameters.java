package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.tools.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoPTrajectoryParameters extends SaveableModuleState
{
   private static final double defaultMinimumDistanceInsidePolygon = 0.01;

   private static final double defaultStepLengthToPutExitCoPOnToes = 0.2;
   private static final double defaultStepHeightToPutExitCoPOnToesSteppingDown = -0.1;
   private static final double defaultStepLengthToPutExitCoPOnToesSteppingDown = 0.1;

   private static final boolean defaultPlanWithExitCMPOnToes = false;
   private static final boolean defaultPlanWithExitCMPOnToesWhenSteppingDown = true;

   private static final double defaultEntryCMPMinX = -0.04;
   private static final double defaultEntryCMPMaxX = 0.03;

   private static final double defaultBallCMPMinX = 0.0;
   private static final double defaultBallCMPMaxX = 0.055;

   private static final double defaultExitCMPMinX = 0.0;
   private static final double defaultExitCMPMaxX = 0.08;

   private static final Vector2DReadOnly defaultEntryCMPOffset = new Vector2D(0.0, -0.005);
   private static final Vector2DReadOnly defaultBallCMPOffset = new Vector2D(0.0, 0.01);
   private static final Vector2DReadOnly defaultExitCMPOffset = new Vector2D(0.0, 0.025);

   private static final double defaultEntryCMPLengthOffsetFactor = 1.0 / 3.0;
   private static final double defaultBallCMPLengthOffsetFactor = 1.0 / 8.0;
   private static final double defaultExitCMPLengthOffsetFactor = 1.0 / 3.0;

   private static final int defaultNumberOfStepsToConsider = 3;

   protected final YoDouble minimumDistanceInsidePolygon;

   protected final YoDouble stepLengthToPutExitCoPOnToes;
   protected final YoDouble stepHeightToPutExitCoPOnToesSteppingDown;
   protected final YoDouble stepLengthToPutExitCoPOnToesSteppingDown;

   protected final YoBoolean planWithExitCMPOnToes;
   protected final YoBoolean planWithExitCMPOnToesWhenSteppingDown;

   protected final YoDouble entryCMPMinX;
   protected final YoDouble entryCMPMaxX;

   protected final YoDouble ballCMPMinX;
   protected final YoDouble ballCMPMaxX;

   protected final YoDouble exitCMPMinX;
   protected final YoDouble exitCMPMaxX;

   protected final YoVector2D entryCMPOffset;
   protected final YoVector2D ballCMPOffset;
   protected final YoVector2D exitCMPOffset;

   protected final YoDouble entryCMPLengthOffsetFactor;
   protected final YoDouble ballCMPLengthOffsetFactor;
   protected final YoDouble exitCMPLengthOffsetFactor;

   protected final YoInteger numberOfStepsToConsider;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public CoPTrajectoryParameters()
   {
      minimumDistanceInsidePolygon = new YoDouble("minimumDistanceInsidePolygon", registry);
      minimumDistanceInsidePolygon.set(defaultMinimumDistanceInsidePolygon);

      stepLengthToPutExitCoPOnToes = new YoDouble("stepLengthToPutExitCoPOnToes", registry);
      stepHeightToPutExitCoPOnToesSteppingDown = new YoDouble("stepHeightToPutExitCoPOnToesSteppingDown", registry);
      stepLengthToPutExitCoPOnToesSteppingDown = new YoDouble("stepLengthToPutExitCoPOnToesSteppingDown", registry);
      stepLengthToPutExitCoPOnToes.set(defaultStepLengthToPutExitCoPOnToes);
      stepHeightToPutExitCoPOnToesSteppingDown.set(defaultStepHeightToPutExitCoPOnToesSteppingDown);
      stepLengthToPutExitCoPOnToesSteppingDown.set(defaultStepLengthToPutExitCoPOnToesSteppingDown);

      planWithExitCMPOnToes = new YoBoolean("planWithExitCMPOnToes", registry);
      planWithExitCMPOnToesWhenSteppingDown = new YoBoolean("planWithExitCMPOnToesWhenSteppingDown", registry);
      planWithExitCMPOnToes.set(defaultPlanWithExitCMPOnToes);
      planWithExitCMPOnToesWhenSteppingDown.set(defaultPlanWithExitCMPOnToesWhenSteppingDown);

      entryCMPMinX = new YoDouble("entryCMPMinX", registry);
      entryCMPMaxX = new YoDouble("entryCMPMaxX", registry);
      entryCMPMinX.set(defaultEntryCMPMinX);
      entryCMPMaxX.set(defaultEntryCMPMaxX);

      ballCMPMinX = new YoDouble("ballCMPMinX", registry);
      ballCMPMaxX = new YoDouble("ballCMPMaxX", registry);
      ballCMPMinX.set(defaultBallCMPMinX);
      ballCMPMaxX.set(defaultBallCMPMaxX);

      exitCMPMinX = new YoDouble("exitCMPMinX", registry);
      exitCMPMaxX = new YoDouble("exitCMPMaxX", registry);
      exitCMPMinX.set(defaultExitCMPMinX);
      exitCMPMaxX.set(defaultExitCMPMaxX);

      entryCMPOffset = new YoVector2D("entryCMPOffset", registry);
      ballCMPOffset = new YoVector2D("ballCMPOffset", registry);
      exitCMPOffset = new YoVector2D("exitCMPOffset", registry);
      entryCMPOffset.set(defaultEntryCMPOffset);
      ballCMPOffset.set(defaultBallCMPOffset);
      exitCMPOffset.set(defaultExitCMPOffset);

      entryCMPLengthOffsetFactor = new YoDouble("entryCMPLengthOffsetFactor", registry);
      ballCMPLengthOffsetFactor = new YoDouble("ballCMPLengthOffsetFactor", registry);
      exitCMPLengthOffsetFactor = new YoDouble("exitCMPLengthOffsetFactor", registry);
      entryCMPLengthOffsetFactor.set(defaultEntryCMPLengthOffsetFactor);
      ballCMPLengthOffsetFactor.set(defaultBallCMPLengthOffsetFactor);
      exitCMPLengthOffsetFactor.set(defaultExitCMPLengthOffsetFactor);

      numberOfStepsToConsider = new YoInteger("numberOfStepsToConsider", registry);
      numberOfStepsToConsider.set(defaultNumberOfStepsToConsider);

      registerDoubleToSave(minimumDistanceInsidePolygon);
      registerDoubleToSave(stepLengthToPutExitCoPOnToes);
      registerDoubleToSave(stepHeightToPutExitCoPOnToesSteppingDown);
      registerDoubleToSave(stepLengthToPutExitCoPOnToesSteppingDown);
      registerDoubleToSave(entryCMPMinX);
      registerDoubleToSave(entryCMPMaxX);
      registerDoubleToSave(ballCMPMinX);
      registerDoubleToSave(ballCMPMaxX);
      registerDoubleToSave(exitCMPMinX);
      registerDoubleToSave(exitCMPMaxX);
      registerBooleanToSave(planWithExitCMPOnToes);
      registerBooleanToSave(planWithExitCMPOnToesWhenSteppingDown);
      SaveableModuleStateTools.registerYoTuple2DToSave(entryCMPOffset, this);
      SaveableModuleStateTools.registerYoTuple2DToSave(ballCMPOffset, this);
      SaveableModuleStateTools.registerYoTuple2DToSave(exitCMPOffset, this);
      registerDoubleToSave(entryCMPLengthOffsetFactor);
      registerDoubleToSave(ballCMPLengthOffsetFactor);
      registerDoubleToSave(exitCMPLengthOffsetFactor);
      registerIntegerToSave(numberOfStepsToConsider);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

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
      return minimumDistanceInsidePolygon.getDoubleValue();
   }

   public boolean getPlanWithExitCMPOnToes()
   {
      return planWithExitCMPOnToes.getBooleanValue();
   }

   public boolean getPlanWithExitCMPOnToesWhenSteppingDown()
   {
      return planWithExitCMPOnToesWhenSteppingDown.getBooleanValue();
   }

   public double getStepLengthToPutExitCoPOnToes()
   {
      return stepLengthToPutExitCoPOnToes.getDoubleValue();
   }

   public double getStepLengthToPutExitCoPOnToesSteppingDown()
   {
      return stepLengthToPutExitCoPOnToesSteppingDown.getDoubleValue();
   }

   public double getStepHeightToPutExitCoPOnToesSteppingDown()
   {
      return stepHeightToPutExitCoPOnToesSteppingDown.getDoubleValue();
   }

   public double getEntryCMPMinX()
   {
      return entryCMPMinX.getDoubleValue();
   }

   public double getEntryCMPMaxX()
   {
      return entryCMPMaxX.getDoubleValue();
   }

   public double getBallCMPMinX()
   {
      return ballCMPMinX.getDoubleValue();
   }

   public double getBallCMPMaxX()
   {
      return ballCMPMaxX.getDoubleValue();
   }

   public double getExitCMPMinX()
   {
      return exitCMPMinX.getDoubleValue();
   }

   public double getExitCMPMaxX()
   {
      return exitCMPMaxX.getDoubleValue();
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
      return entryCMPLengthOffsetFactor.getDoubleValue();
   }

   public double getBallCMPLengthOffsetFactor()
   {
      return ballCMPLengthOffsetFactor.getDoubleValue();
   }

   public double getExitCMPLengthOffsetFactor()
   {
      return exitCMPLengthOffsetFactor.getDoubleValue();
   }

   public int getNumberOfStepsToConsider()
   {
      return numberOfStepsToConsider.getIntegerValue();
   }

   public PlanForToeOffCalculator getPlanForToeOffCalculator()
   {
      return planForToeOffCalculator;
   }
}
