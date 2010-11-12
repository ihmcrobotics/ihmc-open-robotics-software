package us.ihmc.commonWalkingControlModules.filters;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class TorqueTransitionFilter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TorqueTransitionFilter");

   private final DoubleYoVariable tauFilterTime = new DoubleYoVariable("tauFilterTime", registry);
   private final DoubleYoVariable alphaTauFilter = new DoubleYoVariable("alphaTauFilter", registry);

   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> previousTorques = new SideDependentList<EnumMap<LegJointName, DoubleYoVariable>>();

   private final RobotSpecificJointNames robotJointNames;
   private final LegJointName[] legJointNames;

   private final ProcessedOutputsInterface processedOutputs;

   public TorqueTransitionFilter(RobotSpecificJointNames robotJointNames, ProcessedOutputsInterface processedOutputs, YoVariableRegistry yoVariableRegistry)
   {
      this.robotJointNames = robotJointNames;
      this.legJointNames = robotJointNames.getLegJointNames();

      this.processedOutputs = processedOutputs;

      previousTorques.set(RobotSide.LEFT, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));
      previousTorques.set(RobotSide.RIGHT, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));

      for (RobotSide robotSide : RobotSide.values())
      {
         for (LegJointName legJointName : legJointNames)
         {
            String name = "prevTau" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression();
            
            System.out.println(name);
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
      for (RobotSide robotSide : RobotSide.values())
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

      for (RobotSide robotSide : RobotSide.values())
      {
         for (LegJointName legJointName : robotJointNames.getLegJointNames())
         {
            processedOutputs.setDesiredLegJointTorque(robotSide, legJointName,
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
