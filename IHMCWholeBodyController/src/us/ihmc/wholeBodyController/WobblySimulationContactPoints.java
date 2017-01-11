package us.ihmc.wholeBodyController;

import java.util.List;
import java.util.Map;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WobblySimulationContactPoints implements SimulationContactPoints
{

   @Override
   public Map<String, List<Tuple3d>> getContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean useSoftContactPointParameters()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
