package us.ihmc.commonWalkingControlModules.optimalSwing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LegTorqueData
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final List<LegJointName> jointNames;

   
   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointTorques = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   
   private final EnumYoVariable<RobotSide> robotSide = new EnumYoVariable<RobotSide>("robotSide", registry, RobotSide.class);


   private final ArrayList<YoVariable> allVariables;

   public LegTorqueData(LegJointName[] jointNames, YoVariableRegistry parentRegistry)
   {
      this.jointNames = Collections.unmodifiableList(Arrays.asList(jointNames));

      
      for(LegJointName jointName : jointNames)
      {
         desiredJointTorques.put(jointName, new DoubleYoVariable("desired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Torque", registry));
      }
      
      if(parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
      
      allVariables = registry.getAllVariablesIncludingDescendants();

   }
   
   public RobotSide getRobotSide()
   {
      return robotSide.getEnumValue();
   }
   
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
   }
   
   public double getDesiredJointTorque(LegJointName jointName)
   {
      return desiredJointTorques.get(jointName).getDoubleValue();
   }
   
   public void setDesiredJointTorque(LegJointName jointName, double value)
   {
      desiredJointTorques.get(jointName).set(value);
   }
   

   public List<LegJointName> getJointNames()
   {
      return jointNames;
   } 
   
   public ArrayList<YoVariable> getAllVariables()
   {
      return allVariables;
   }
   
}
