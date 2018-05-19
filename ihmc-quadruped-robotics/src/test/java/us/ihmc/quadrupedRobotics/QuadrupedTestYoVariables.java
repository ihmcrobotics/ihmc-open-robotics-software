package us.ihmc.quadrupedRobotics;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class QuadrupedTestYoVariables
{
   private final YoDouble yoTime;

   private final YoDouble robotBodyX;
   private final YoDouble robotBodyY;
   private final YoDouble robotBodyZ;
   private final YoDouble robotBodyYaw;

   private final QuadrantDependentList<YoBoolean> controllerFootSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoBoolean> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionXs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionYs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionZs = new QuadrantDependentList<>();
   
   private final YoBoolean limitJointTorques;
   
   public QuadrupedTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (YoDouble) scs.getVariable("t");

      robotBodyX = (YoDouble) scs.getVariable("q_x");
      robotBodyY = (YoDouble) scs.getVariable("q_y");
      robotBodyZ = (YoDouble) scs.getVariable("q_z");
      robotBodyYaw = (YoDouble) scs.getVariable("q_yaw");
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerFootSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch_controllerThinksHasTouchedDown"));
         footSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected"));
         solePositionXs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionX"));
         solePositionYs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionY"));
         solePositionZs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionZ"));
      }
      
      limitJointTorques = (YoBoolean) scs.getVariable("limitJointTorques");
   }

   public YoDouble getRobotBodyX()
   {
      return robotBodyX;
   }

   public YoDouble getRobotBodyY()
   {
      return robotBodyY;
   }

   public YoDouble getRobotBodyZ()
   {
      return robotBodyZ;
   }

   public YoDouble getRobotBodyYaw()
   {
      return robotBodyYaw;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public QuadrantDependentList<YoBoolean> getControllerFootSwitches()
   {
      return controllerFootSwitches;
   }

   public QuadrantDependentList<YoBoolean> getFootSwitches()
   {
      return footSwitches;
   }

   public QuadrantDependentList<YoDouble> getSolePositionXs()
   {
      return solePositionXs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionYs()
   {
      return solePositionYs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionZs()
   {
      return solePositionZs;
   }

   public YoBoolean getLimitJointTorques()
   {
      return limitJointTorques;
   }
}
