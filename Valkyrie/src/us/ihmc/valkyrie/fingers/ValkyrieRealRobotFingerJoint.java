package us.ihmc.valkyrie.fingers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.CommonNames;

public enum ValkyrieRealRobotFingerJoint
{
   /**
    * Map enums to RoboNet names.
    */
   ThumbRoll("ThumbRoll"), Thumb("ThumbPitch1"), Index("IndexFingerPitch1"), Middle("MiddleFingerPitch1"), 
   Pinky("PinkyPitch1");

   public static final ValkyrieRealRobotFingerJoint[] values = ValkyrieRealRobotFingerJoint.values();
   
   private String name;

   ValkyrieRealRobotFingerJoint(String name)
   {
      this.name = name;
   }
   
   public DoubleYoVariable getRelatedControlVariable(RobotSide robotSide, YoVariableRegistry registry)
   {
      return (DoubleYoVariable) registry.getVariable(robotSide.getCamelCaseNameForMiddleOfExpression() + name + CommonNames.q_d);
   }

   @Override
   public String toString()
   {
      return name;
   }
}
