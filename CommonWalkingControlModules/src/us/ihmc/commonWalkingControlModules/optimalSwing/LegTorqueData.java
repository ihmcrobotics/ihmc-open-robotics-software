package us.ihmc.commonWalkingControlModules.optimalSwing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;


public class LegTorqueData
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final List<LegJointName> jointNames;

   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointPositions = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointVelocities = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointAccelerations = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   
   private final EnumYoVariable<RobotSide> robotSide = new EnumYoVariable<RobotSide>("robotSide", registry, RobotSide.class);

   private final ArrayList<YoVariable<?>> allVariables;
   private final BooleanYoVariable dataValid = new BooleanYoVariable("dataValid", registry);
   private final DoubleYoVariable timeTakenForSwingOptimization = new DoubleYoVariable("timeTakenForSwingOptimization", registry);

   public LegTorqueData(LegJointName[] jointNames, YoVariableRegistry parentRegistry)
   {
      this.jointNames = Collections.unmodifiableList(Arrays.asList(jointNames));

      
      for(LegJointName jointName : jointNames)
      {
         desiredJointPositions.put(jointName, new DoubleYoVariable("desired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Positon", registry));
         desiredJointVelocities.put(jointName, new DoubleYoVariable("desired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Velocity", registry));
         desiredJointAccelerations.put(jointName, new DoubleYoVariable("desired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Acceleration", registry));
      }
      
      if(parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
      
      allVariables = registry.getAllVariablesIncludingDescendants();

   }
   public void setTimeTakenForSwingOptimization(double timeTaken)
   {
      timeTakenForSwingOptimization.set(timeTaken);
   }
   
   public double getTimeTakenForSwingOptimization()
   {
      return timeTakenForSwingOptimization.getDoubleValue();
   }
   
   public RobotSide getRobotSide()
   {
      return robotSide.getEnumValue();
   }
   
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
   }
   
   public double getDesiredJointPosition(LegJointName jointName)
   {
      return desiredJointPositions.get(jointName).getDoubleValue();
   }
   
   public void setDesiredJointPosition(LegJointName jointName, double value)
   {
      desiredJointPositions.get(jointName).set(value);
   }
   
   public double getDesiredJointVelocity(LegJointName jointName)
   {
      return desiredJointVelocities.get(jointName).getDoubleValue();
   }

   public void setDesiredJointVelocity(LegJointName jointName, double value)
   {
      desiredJointVelocities.get(jointName).set(value);
   }
   
   public double getDesiredJointAcceleration(LegJointName jointName)
   {
      return desiredJointAccelerations.get(jointName).getDoubleValue();
   }
   
   public void setDesiredJointAcceleration(LegJointName jointName, double value)
   {
      desiredJointAccelerations.get(jointName).set(value);
   }

   public List<LegJointName> getJointNames()
   {
      return jointNames;
   } 
   
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return allVariables;
   }

   public void setDataValid(boolean dataValid)
   {
      this.dataValid.set(dataValid);
   }

   public boolean isDataValid()
   {
      return dataValid.getBooleanValue();
   }
   
}
