package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.ui.UIAuxiliaryRobotData;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.util.function.ToDoubleFunction;

public class ValkyrieUIAuxiliaryData implements UIAuxiliaryRobotData
{
   private static final Vector3D rootJointToMidFootOffset = new Vector3D(0.0359987, 0.0, -0.9900972);
   private static final ToDoubleFunction<String> setPointMap = new ValkyrieRobotModel(RobotTarget.SCS).getHighLevelControllerParameters().getStandPrepParameters()::getSetpoint;


   @Override
   public Vector3D getRootJointToMidFootOffset()
   {
      return rootJointToMidFootOffset;
   }

   @Override
   public ToDoubleFunction<String> getDefaultJointAngleMap()
   {
      return setPointMap;
   }

   @Override
   public SideDependentList<double[]> getArmsInJointAngles()
   {
      return null;
   }
}
