package us.ihmc.commonWalkingControlModules.optimalSwing;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SwingParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SwingParameters");
   private final BooleanYoVariable currentlyInSwing = new BooleanYoVariable("currentlyInSwing", registry);
   private final EnumYoVariable<RobotSide> robotSide = new EnumYoVariable<RobotSide>("robotSide", registry, RobotSide.class);
   private final DoubleYoVariable swingTimeRemaining = new DoubleYoVariable("swingTimeRemaining", registry);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredJointPositionsAtEndOfStep = new SideDependentList<EnumMap<LegJointName,DoubleYoVariable>>();
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredJointVelocitiesAtEndOfStep = new SideDependentList<EnumMap<LegJointName,DoubleYoVariable>>();
   

   private final LegJointName[] legJointNames;

   private final ArrayList<YoVariable> allVariables;
   
   public SwingParameters(LegJointName[] legJointNames, YoVariableRegistry parentRegistry)
   {
      this.legJointNames = legJointNames;
      
      for(RobotSide robotSide : RobotSide.values())
      {
         desiredJointPositionsAtEndOfStep.put(robotSide, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));
         desiredJointVelocitiesAtEndOfStep.put(robotSide, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));
         
         for(LegJointName legJointName : legJointNames)
         {
            desiredJointPositionsAtEndOfStep.get(robotSide).put(legJointName, new DoubleYoVariable("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression()+"Position", registry));
            desiredJointVelocitiesAtEndOfStep.get(robotSide).put(legJointName, new DoubleYoVariable("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression()+"Velocity", registry));
         }
         
      }
      
      allVariables = registry.getAllVariablesIncludingDescendants();
      if(parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public double getSwingTimeRemaining()
   {
      return swingTimeRemaining.getDoubleValue();
   }

   public void setSwingTimeRemaining(double swingTimeRemaining)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
   }

   public RobotSide getRobotSide()
   {
      return robotSide.getEnumValue();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
   }
   
   public void setDesiredJointPosition(RobotSide robotSide, LegJointName jointName, double value)
   {
      desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).set(value);
   }
   
   public double getDesiredJointPosition(RobotSide robotSide, LegJointName jointName)
   {
      return desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).getDoubleValue();
   }
   
   public void getDesiredJointPositions(LegJointPositions legJointPositionsToPack)
   {
      RobotSide robotSide = legJointPositionsToPack.getRobotSide();
      for(LegJointName jointName : legJointNames)
      {
         legJointPositionsToPack.setJointPosition(jointName, desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).getDoubleValue());
      }
   }
   
   public void getDesiredJointVelocities(LegJointVelocities legJointVelocitiesToPack)
   {
      RobotSide robotSide = legJointVelocitiesToPack.getRobotSide();
      for(LegJointName jointName : legJointNames)
      {
         legJointVelocitiesToPack.setJointVelocity(jointName, desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).getDoubleValue());
      }
   }
   
   
   public void setDesiredJointVelocity(RobotSide robotSide, LegJointName jointName, double value)
   {
      desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).set(value);
   }
   
   public double getDesiredJointVelocity(RobotSide robotSide, LegJointName jointName)
   {
      return desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).getDoubleValue();
   }
   
   public ArrayList<YoVariable> getAllVariables()
   {
      return allVariables;
   }
   
   public void setCurrentlyInSwing(boolean currentlyInSwing)
   {
      this.currentlyInSwing.set(currentlyInSwing);
   }
   
   public boolean isCurrentlyInSwing()
   {
      return currentlyInSwing.getBooleanValue();
   }
}
