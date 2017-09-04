package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FeetLoadedTransition implements StateTransitionCondition
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double MINIMUM_FORCE_TO_SWITCH = 50.0;

   private final SideDependentList<ForceSensorDataReadOnly> footSensors = new SideDependentList<>();

   private final YoDouble prepLeftFootFz;
   private final YoDouble prepRightFootFz;
   private final SimpleMovingAverageFilteredYoVariable prepLeftFootFzAverage;
   private final SimpleMovingAverageFilteredYoVariable prepRightFootFzAverage;

   public FeetLoadedTransition(ForceSensorDataHolderReadOnly forceSensorDataHolder, SideDependentList<String> feetContactSensors, double controlDT,
                               YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
         footSensors.put(robotSide, forceSensorDataHolder.getByName(feetContactSensors.get(robotSide)));

      int windowSize = (int) Math.floor(14.0 / controlDT);
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
      return (prepLeftFootFzAverage.getDoubleValue() + prepRightFootFzAverage.getDoubleValue() > 2.0 * MINIMUM_FORCE_TO_SWITCH);
   }

   @Override
   public boolean checkCondition()
   {
      return areFeetLoaded();
   }
}
