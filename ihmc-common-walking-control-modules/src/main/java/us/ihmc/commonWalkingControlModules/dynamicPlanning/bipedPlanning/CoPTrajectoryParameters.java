package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector2D;
import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.tools.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
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

   private static final double defaultDurationForContinuityMaintenanceSegment = 0.2;
   private static final double defaultSafeDistanceFromCoPToSupportEdgesWhenSteppingDown = 0.0;
   private static final double defaultExitCoPForwardSafetyMarginOnToes = 1.6e-2;

   private static final Vector2DReadOnly defaultEntryCMPOffset = new Vector2D(0.0, -0.005);
   private static final Vector2DReadOnly defaultBallCMPOffset = new Vector2D(0.0, 0.01);
   private static final Vector2DReadOnly defaultExitCMPOffset = new Vector2D(0.0, 0.025);

   private static final double defaultEntryCMPLengthOffsetFactor = 1.0 / 3.0;
   private static final double defaultBallCMPLengthOffsetFactor = 1.0 / 8.0;
   private static final double defaultExitCMPLengthOffsetFactor = 1.0 / 3.0;

   private static final int defaultNumberOfStepsToConsider = 3;

   protected final DoubleParameter minimumDistanceInsidePolygon;

   protected final DoubleParameter stepLengthToPutExitCoPOnToes;
   protected final DoubleParameter stepHeightToPutExitCoPOnToesSteppingDown;
   protected final DoubleParameter stepLengthToPutExitCoPOnToesSteppingDown;

   protected final BooleanParameter planWithExitCMPOnToes;
   protected final BooleanParameter planWithExitCMPOnToesWhenSteppingDown;

   protected final DoubleParameter entryCMPMinX;
   protected final DoubleParameter entryCMPMaxX;

   protected final DoubleParameter ballCMPMinX;
   protected final DoubleParameter ballCMPMaxX;

   protected final DoubleParameter exitCMPMinX;
   protected final DoubleParameter exitCMPMaxX;

   protected final ParameterVector2D entryCMPOffset;
   protected final ParameterVector2D ballCMPOffset;
   protected final ParameterVector2D exitCMPOffset;

   protected final DoubleParameter entryCMPLengthOffsetFactor;
   protected final DoubleParameter ballCMPLengthOffsetFactor;
   protected final DoubleParameter exitCMPLengthOffsetFactor;

   protected final IntegerParameter numberOfStepsToConsider;

   private final DoubleParameter durationForContinuityMaintenanceSegment;
   private final DoubleParameter safeDistanceFromCoPToSupportEdgesWhenSteppingDown;
   private final DoubleParameter exitCoPForwardSafetyMarginOnToes;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public CoPTrajectoryParameters()
   {
      minimumDistanceInsidePolygon = new DoubleParameter("minimumDistanceInsidePolygon", registry, defaultMinimumDistanceInsidePolygon);

      stepLengthToPutExitCoPOnToes = new DoubleParameter("stepLengthToPutExitCoPOnToes", registry, defaultStepLengthToPutExitCoPOnToes);
      stepHeightToPutExitCoPOnToesSteppingDown = new DoubleParameter("stepHeightToPutExitCoPOnToesSteppingDown", registry, defaultStepHeightToPutExitCoPOnToesSteppingDown);
      stepLengthToPutExitCoPOnToesSteppingDown = new DoubleParameter("stepLengthToPutExitCoPOnToesSteppingDown", registry, defaultStepLengthToPutExitCoPOnToesSteppingDown);

      planWithExitCMPOnToes = new BooleanParameter("planWithExitCMPOnToes", registry, defaultPlanWithExitCMPOnToes);
      planWithExitCMPOnToesWhenSteppingDown = new BooleanParameter("planWithExitCMPOnToesWhenSteppingDown", registry,defaultPlanWithExitCMPOnToesWhenSteppingDown);

      entryCMPMinX = new DoubleParameter("entryCMPMinX", registry, defaultEntryCMPMinX);
      entryCMPMaxX = new DoubleParameter("entryCMPMaxX", registry, defaultEntryCMPMaxX);

      ballCMPMinX = new DoubleParameter("ballCMPMinX", registry, defaultBallCMPMinX);
      ballCMPMaxX = new DoubleParameter("ballCMPMaxX", registry, defaultBallCMPMaxX);

      exitCMPMinX = new DoubleParameter("exitCMPMinX", registry, defaultExitCMPMinX);
      exitCMPMaxX = new DoubleParameter("exitCMPMaxX", registry, defaultExitCMPMaxX);

      entryCMPOffset = new ParameterVector2D("entryCMPOffset", defaultEntryCMPOffset, registry);
      ballCMPOffset = new ParameterVector2D("ballCMPOffset", defaultBallCMPOffset, registry);
      exitCMPOffset = new ParameterVector2D("exitCMPOffset", defaultExitCMPOffset, registry);

      entryCMPLengthOffsetFactor = new DoubleParameter("entryCMPLengthOffsetFactor", registry, defaultEntryCMPLengthOffsetFactor);
      ballCMPLengthOffsetFactor = new DoubleParameter("ballCMPLengthOffsetFactor", registry, defaultBallCMPLengthOffsetFactor);
      exitCMPLengthOffsetFactor = new DoubleParameter("exitCMPLengthOffsetFactor", registry, defaultExitCMPLengthOffsetFactor);

      numberOfStepsToConsider = new IntegerParameter("numberOfStepsToConsider", registry, defaultNumberOfStepsToConsider);

      durationForContinuityMaintenanceSegment = new DoubleParameter("durationForContinuityMaintenanceSegment", registry, defaultDurationForContinuityMaintenanceSegment);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new DoubleParameter("safeDistanceFromCoPToSupportEdgesWhenSteppingDown", registry, defaultSafeDistanceFromCoPToSupportEdgesWhenSteppingDown);
      exitCoPForwardSafetyMarginOnToes = new DoubleParameter("exitCoPForwardSafetyMarginOnToes", registry, defaultExitCoPForwardSafetyMarginOnToes);

      registerVariableToSave(minimumDistanceInsidePolygon);
      registerVariableToSave(stepLengthToPutExitCoPOnToes);
      registerVariableToSave(stepHeightToPutExitCoPOnToesSteppingDown);
      registerVariableToSave(stepLengthToPutExitCoPOnToesSteppingDown);
      registerVariableToSave(entryCMPMinX);
      registerVariableToSave(entryCMPMaxX);
      registerVariableToSave(ballCMPMinX);
      registerVariableToSave(ballCMPMaxX);
      registerVariableToSave(exitCMPMinX);
      registerVariableToSave(exitCMPMaxX);
      registerVariableToSave(planWithExitCMPOnToes);
      registerVariableToSave(planWithExitCMPOnToesWhenSteppingDown);
      registerVariableToSave(entryCMPOffset.getXParameter());
      registerVariableToSave(entryCMPOffset.getYParameter());
      registerVariableToSave(ballCMPOffset.getXParameter());
      registerVariableToSave(ballCMPOffset.getYParameter());
      registerVariableToSave(exitCMPOffset.getXParameter());
      registerVariableToSave(exitCMPOffset.getYParameter());
      registerVariableToSave(entryCMPLengthOffsetFactor);
      registerVariableToSave(ballCMPLengthOffsetFactor);
      registerVariableToSave(exitCMPLengthOffsetFactor);
      registerVariableToSave(numberOfStepsToConsider);
      registerVariableToSave(durationForContinuityMaintenanceSegment);
      registerVariableToSave(safeDistanceFromCoPToSupportEdgesWhenSteppingDown);
      registerVariableToSave(exitCoPForwardSafetyMarginOnToes);
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
      return minimumDistanceInsidePolygon.getValue();
   }

   public boolean getPlanWithExitCMPOnToes()
   {
      return planWithExitCMPOnToes.getValue();
   }

   public boolean getPlanWithExitCMPOnToesWhenSteppingDown()
   {
      return planWithExitCMPOnToesWhenSteppingDown.getValue();
   }

   public double getStepLengthToPutExitCoPOnToes()
   {
      return stepLengthToPutExitCoPOnToes.getValue();
   }

   public double getStepLengthToPutExitCoPOnToesSteppingDown()
   {
      return stepLengthToPutExitCoPOnToesSteppingDown.getValue();
   }

   public double getStepHeightToPutExitCoPOnToesSteppingDown()
   {
      return stepHeightToPutExitCoPOnToesSteppingDown.getValue();
   }

   public double getEntryCMPMinX()
   {
      return entryCMPMinX.getValue();
   }

   public double getEntryCMPMaxX()
   {
      return entryCMPMaxX.getValue();
   }

   public double getBallCMPMinX()
   {
      return ballCMPMinX.getValue();
   }

   public double getBallCMPMaxX()
   {
      return ballCMPMaxX.getValue();
   }

   public double getExitCMPMinX()
   {
      return exitCMPMinX.getValue();
   }

   public double getExitCMPMaxX()
   {
      return exitCMPMaxX.getValue();
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
      return entryCMPLengthOffsetFactor.getValue();
   }

   public double getBallCMPLengthOffsetFactor()
   {
      return ballCMPLengthOffsetFactor.getValue();
   }

   public double getExitCMPLengthOffsetFactor()
   {
      return exitCMPLengthOffsetFactor.getValue();
   }

   public int getNumberOfStepsToConsider()
   {
      return numberOfStepsToConsider.getValue();
   }

   public double getDurationForContinuityMaintenanceSegment()
   {
      return durationForContinuityMaintenanceSegment.getValue();
   }

   public double getSafeDistanceFromCoPToSupportEdgesWhenSteppingDown()
   {
      return safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getValue();
   }

   public double getExitCoPForwardSafetyMarginOnToes()
   {
      return exitCoPForwardSafetyMarginOnToes.getValue();
   }

   public PlanForToeOffCalculator getPlanForToeOffCalculator()
   {
      return planForToeOffCalculator;
   }
}
