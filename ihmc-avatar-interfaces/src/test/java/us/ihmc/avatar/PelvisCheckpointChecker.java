package us.ihmc.avatar;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class PelvisCheckpointChecker extends SimpleRobotController
{
   private final HumanoidFloatingRootJointRobot humanoidRobotModel;

   private ArrayList<YoBoolean> footCheckPointFlag;
   private ArrayList<BoundingBox3D> footCheckPoint;
   private Point3D position;
   private int footStepCheckPointIndex;

   private YoVariableRegistry circleWalkRegistry;

   public PelvisCheckpointChecker(DRCSimulationTestHelper drcSimulationTestHelper)
   {
      humanoidRobotModel = drcSimulationTestHelper.getRobot();
      position = new Point3D();
      footStepCheckPointIndex = 0;

      circleWalkRegistry = new YoVariableRegistry("WalkingTest");
      drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
   }

   @Override
   public void doControl()
   {
      checkFootCheckPoints();
   }

   private void checkFootCheckPoints()
   {
      humanoidRobotModel.getRootJoint().getPosition(position);
      if (footStepCheckPointIndex < footCheckPoint.size() && footCheckPoint.get(footStepCheckPointIndex).isInsideInclusive(position))
      {
         footCheckPointFlag.get(footStepCheckPointIndex).set(true);
         if (footStepCheckPointIndex < footCheckPoint.size())
            footStepCheckPointIndex++;
      }
   }

   public void setFootStepCheckPoints(ArrayList<Point3D> locations, double xRange, double yRange)
   {
      footCheckPointFlag = new ArrayList<>(locations.size());
      footCheckPoint = new ArrayList<>(locations.size());

      for (int i = 0; i < locations.size(); i++)
      {
         Point3D minBound = new Point3D(locations.get(i));
         minBound.add(-xRange / 2.0, -yRange / 2.0, -10.0);
         Point3D maxBound = new Point3D(locations.get(i));
         maxBound.add(xRange / 2.0, yRange / 2.0, 10.0);
         footCheckPoint.add(new BoundingBox3D(minBound, maxBound));
         YoBoolean newFlag = new YoBoolean("FootstepCheckPointFlag" + Integer.toString(i), circleWalkRegistry);
         footCheckPointFlag.add(newFlag);
      }
   }

   public void assertCheckpointsReached()
   {
      boolean reachedAllCheckpoints = true;
      for (int i = 0; i < footCheckPointFlag.size(); i++)
      {
         boolean reachedCheckpoint = footCheckPointFlag.get(i).getBooleanValue();
         if (!reachedCheckpoint)
         {
            reachedAllCheckpoints = false;
            Point3DBasics center = new Point3D();
            footCheckPoint.get(i).getCenterPoint(center);
            PrintTools.info("Pelvis did not reach checkpoint " + i + " at position " + center);
         }
      }
      assertTrue(reachedAllCheckpoints);
   }
}