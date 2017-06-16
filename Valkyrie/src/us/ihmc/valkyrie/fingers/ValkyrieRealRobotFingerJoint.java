package us.ihmc.valkyrie.fingers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
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
   
   public YoDouble getRelatedControlVariable(RobotSide robotSide, YoVariableRegistry registry)
   {
      return (YoDouble) registry.getVariable(robotSide.getCamelCaseNameForMiddleOfExpression() + name + CommonNames.q_d);
   }

   @Override
   public String toString()
   {
      return name;
   }
}
