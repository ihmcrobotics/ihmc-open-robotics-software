package us.ihmc.perception.tools;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class PerceptionFilterTools
{
   public static void filterCollidingPlanarRegions(FramePlanarRegionsList regionsInSensor, CollidingScanRegionFilter filter)
   {
      int i = 0;
      while (i < regionsInSensor.getPlanarRegionsList().getNumberOfPlanarRegions())
      {
         PlanarRegion regionInWorld = regionsInSensor.getPlanarRegionsList().getPlanarRegion(i).copy();
         regionInWorld.applyTransform(regionsInSensor.getSensorToWorldFrameTransform());
         boolean collision = !filter.test(i, regionInWorld);
         if (collision)
         {
            regionsInSensor.getPlanarRegionsList().pollPlanarRegion(i);
         }
         else
            i++;
      }
   }

   public static void applyCollisionFilter(FramePlanarRegionsList framePlanarRegionsList, CollidingScanRegionFilter collisionFilter)
   {
      collisionFilter.update();
      PlanarRegionsList planarRegionsList = framePlanarRegionsList.getPlanarRegionsList();

      List<PlanarRegion> filteredPlanarRegions = planarRegionsList.getPlanarRegionsAsList().parallelStream()
                                                                  .filter(region -> {
                                                                     PlanarRegion regionInWorld = region.copy();
                                                                     regionInWorld.applyTransform(framePlanarRegionsList.getSensorToWorldFrameTransform());
                                                                     return collisionFilter.test(0, regionInWorld);
                                                                  }).toList();

      framePlanarRegionsList.getPlanarRegionsList().clear();
      framePlanarRegionsList.getPlanarRegionsList().addPlanarRegions(filteredPlanarRegions);
   }

   public static CollidingScanRegionFilter createHumanoidShinCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      CollisionShapeTester shapeTester = new CollisionShapeTester(fullRobotModel, collisionBoxProvider);
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      return new CollidingScanRegionFilter(shapeTester);
   }
}