package us.ihmc.footstepPlanning.ui;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.ToDoubleFunction;

public interface UIAuxiliaryRobotData
{
   Vector3D getRootJointToMidFootOffset();

   ToDoubleFunction<String> getDefaultJointAngleMap();

   SideDependentList<double[]> getArmsInJointAngles();
}
