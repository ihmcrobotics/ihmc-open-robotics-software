package us.ihmc.commonWalkingControlModules.filters;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;



public class TorqueTransitionFilter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TorqueTransitionFilter");

   private final DoubleYoVariable tauFilterTime = new DoubleYoVariable("tauFilterTime", registry);
   private final DoubleYoVariable alphaTauFilter = new DoubleYoVariable("alphaTauFilter", registry);

   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> previousTorques = SideDependentList.createListOfEnumMaps(LegJointName.class);

   private final RobotSpecificJointNames robotJointNames;
   private final LegJointName[] legJointNames;

   private final ProcessedOutputsInterface processedOutputs;

   public TorqueTransitionFilter(RobotSpecificJointNames robotJointNames, ProcessedOutputsInterface processedOutputs, YoVariableRegistry yoVariableRegistry)
   {
      this.robotJointNames = robotJointNames;
      this.legJointNames = robotJointNames.getLegJointNames();

      this.processedOutputs = processedOutputs;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            String name = "prevTau" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression();
            
            DoubleYoVariable previousTorque = new DoubleYoVariable(name, "Previous Torque", registry);

            previousTorques.get(robotSide).put(legJointName, previousTorque);

         }
      }

      tauFilterTime.set(0.15); // Default value. But user should probably call setTauFilterTime();

      if (yoVariableRegistry != null)
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   public void rememberPreviousTorques()
   {
      // Remember previous torques:
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            DoubleYoVariable previousTorque = previousTorques.get(robotSide).get(legJointName);
            previousTorque.set(processedOutputs.getDesiredLegJointTorque(robotSide, legJointName));
         }
      }
   }


   public void updateTorques(double timeSinceTransition)
   {
      alphaTauFilter.set(1.0 - (timeSinceTransition / tauFilterTime.getDoubleValue()));
      if (alphaTauFilter.getDoubleValue() < 0.0)
         alphaTauFilter.set(0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : robotJointNames.getLegJointNames())
         {
            processedOutputs.setLegJointTau(robotSide, legJointName,
                    alphaTauFilter.getDoubleValue() * previousTorques.get(robotSide).get(legJointName).getDoubleValue()
                    + (1.0 - alphaTauFilter.getDoubleValue()) * processedOutputs.getDesiredLegJointTorque(robotSide, legJointName));
         }
      }

   }

   public void setTauFilterTime(double tauFilterTime)
   {
      this.tauFilterTime.set(tauFilterTime);
   }
}
