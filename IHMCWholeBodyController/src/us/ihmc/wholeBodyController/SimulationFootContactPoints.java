package us.ihmc.wholeBodyController;

import java.util.List;
import java.util.Map;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface SimulationFootContactPoints
{
   public Map<String, List<Tuple3d>> getContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms);

   public boolean useSoftContactPointParameters();
}