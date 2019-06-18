package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedFeetLoadedTransition implements StateTransitionCondition
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double MINIMUM_WEIGHT_FRACTION = 1.0 / 12.0;
   private static final double TIME_WINDOW = 3.0;

   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   private final YoBoolean areFeetLoaded;
   private final YoDouble weightPerFootForLoaded;

   private final QuadrantDependentList<YoDouble> prepFootFzs = new QuadrantDependentList<>();
   private final QuadrantDependentList<SimpleMovingAverageFilteredYoVariable> prepFootFzAverages = new QuadrantDependentList<>();

   public QuadrupedFeetLoadedTransition(QuadrantDependentList<FootSwitchInterface> footSwitches, double controlDT, double gravityZ, double totalMass,
                                        YoVariableRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;

      int windowSize = (int) Math.floor(TIME_WINDOW / controlDT);

      areFeetLoaded = new YoBoolean("areFeetLoaded", registry);
      weightPerFootForLoaded = new YoDouble("weightPerFootForLoaded", registry);
      weightPerFootForLoaded.set(gravityZ * totalMass * MINIMUM_WEIGHT_FRACTION);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoDouble prepFootFz = new YoDouble("prep" + robotQuadrant.getCamelCaseName() + "FootFz", registry);
         SimpleMovingAverageFilteredYoVariable prepFootFzAverage = new SimpleMovingAverageFilteredYoVariable(
               "prep" + robotQuadrant.getCamelCaseName() + "FootFzAverage", windowSize, prepFootFz, registry);

         prepFootFzs.put(robotQuadrant, prepFootFz);
         prepFootFzAverages.put(robotQuadrant, prepFootFzAverage);
      }

      parentRegistry.addChild(registry);
   }

   private final Wrench temporaryFootWrench = new Wrench();

   private boolean areFeetLoaded()
   {
      temporaryFootWrench.setToZero(ReferenceFrame.getWorldFrame());

      double averageWeight = 0.0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoDouble prepFootFz = prepFootFzs.get(robotQuadrant);
         SimpleMovingAverageFilteredYoVariable prepFootFzAverage = prepFootFzAverages.get(robotQuadrant);

         footSwitches.get(robotQuadrant).computeAndPackFootWrench(temporaryFootWrench);

         temporaryFootWrench.changeFrame(ReferenceFrame.getWorldFrame());

         prepFootFz.set(temporaryFootWrench.getLinearPartZ());
         prepFootFzAverage.update();

         averageWeight += prepFootFzAverage.getDoubleValue();
      }
      areFeetLoaded.set((averageWeight > 2.0 * weightPerFootForLoaded.getDoubleValue()));

      return areFeetLoaded.getBooleanValue();
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      return areFeetLoaded();
   }
}
