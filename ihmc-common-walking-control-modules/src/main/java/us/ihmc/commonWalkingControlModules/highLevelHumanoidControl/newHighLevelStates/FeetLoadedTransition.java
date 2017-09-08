package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FeetLoadedTransition implements StateTransitionCondition
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double MINIMUM_WEIGHT_FRACTION = 1.0 / 4.0;
   private static final double TIME_WINDOW = 5.0;

   private final SideDependentList<ForceSensorDataReadOnly> footSensors = new SideDependentList<>();

   private final YoBoolean areFeetLoaded;
   private final YoDouble weightPerFootForLoaded;

   private final YoDouble prepLeftFootFz;
   private final YoDouble prepRightFootFz;
   private final SimpleMovingAverageFilteredYoVariable prepLeftFootFzAverage;
   private final SimpleMovingAverageFilteredYoVariable prepRightFootFzAverage;

   public FeetLoadedTransition(ForceSensorDataHolderReadOnly forceSensorDataHolder, SideDependentList<String> feetContactSensors,
                               double controlDT, double gravityZ, double totalMass, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
         footSensors.put(robotSide, forceSensorDataHolder.getByName(feetContactSensors.get(robotSide)));

      int windowSize = (int) Math.floor(TIME_WINDOW / controlDT);

      areFeetLoaded = new YoBoolean("areFeetLoaded", registry);
      weightPerFootForLoaded = new YoDouble("weightPerFootForLoaded", registry);
      weightPerFootForLoaded.set(gravityZ * totalMass * MINIMUM_WEIGHT_FRACTION);

      prepLeftFootFz = new YoDouble("prepLeftFootFz", registry);
      prepRightFootFz = new YoDouble("prepRightFootFz", registry);
      prepLeftFootFzAverage = new SimpleMovingAverageFilteredYoVariable("prepLeftFootFzAverage", windowSize, prepLeftFootFz, registry);
      prepRightFootFzAverage = new SimpleMovingAverageFilteredYoVariable("prepRightFootFzAverage", windowSize, prepRightFootFz, registry);

      parentRegistry.addChild(registry);
   }

   private final Wrench temporaryFootWrench = new Wrench();

   private boolean areFeetLoaded()
   {
      footSensors.get(RobotSide.LEFT).getWrench(temporaryFootWrench);
      prepLeftFootFz.set(temporaryFootWrench.getLinearPartZ());
      footSensors.get(RobotSide.RIGHT).getWrench(temporaryFootWrench);
      prepRightFootFz.set(temporaryFootWrench.getLinearPartZ());
      prepLeftFootFzAverage.update();
      prepRightFootFzAverage.update();
      areFeetLoaded.set((prepLeftFootFzAverage.getDoubleValue() + prepRightFootFzAverage.getDoubleValue() > 2.0 * weightPerFootForLoaded.getDoubleValue()));

      return areFeetLoaded.getBooleanValue();
   }

   @Override
   public boolean checkCondition()
   {
      return areFeetLoaded();
   }
}
