package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import java.util.EnumMap;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;


public interface ArmReferenceForPDControlModule
{
   public abstract void setDesiredJointPositionOnBothRobotSides(ArmJointName armJointName, double value);
   
}
