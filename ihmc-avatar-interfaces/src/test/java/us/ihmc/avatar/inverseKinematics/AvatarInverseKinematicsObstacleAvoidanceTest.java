package us.ihmc.avatar.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.badlogic.gdx.math.collision.BoundingBox;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionBox;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class AvatarInverseKinematicsObstacleAvoidanceTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private YoVariableRegistry testRegistry = new YoVariableRegistry("InverseKinematicsTestRegistry");
   private YoGraphicsListRegistry graphicsRegistry = new YoGraphicsListRegistry();

   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public abstract boolean keepSCSUp();

   public abstract int getNumberOfObstacles();

   public abstract boolean specifyObstacle();

   public abstract List<CollisionShape> getListOfObstacles();

   public abstract BoundingBox getWorkspaceBoundsForCreatingObstacles();

   private List<CollisionShape> obstacleList = new ArrayList<>();

   /**
    * There is no need to implement this method is {@code specifyObstaclePositions()} returns false
    * @return
    */
   public abstract List<FramePoint3D> getObstaclePositions();

   @Before
   public void setupTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = new SimulationTestingParameters();
      simulationTestingParameters.setKeepSCSUp(keepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
   }

   @After
   public void cleanTest()
   {
      cleanTestVariables();
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void cleanTestVariables()
   {
      obstacleList.clear();

   }

   private void createObstacles()
   {
      if (specifyObstacle())
      {

      }
      else
      {
         BoundingBox workspace = getWorkspaceBoundsForCreatingObstacles();

         int numberOfObstaclesToCreate = getNumberOfObstacles();
         for (int i = 0; i < numberOfObstaclesToCreate; i++)
         {
            RigidBodyTransform obstaclePose = new RigidBodyTransform();
            obstaclePose.setTranslation(10, 10, 10);
         }
      }
   }

   private void createVisualizationForObstacles(List<CollisionShape> obstacleList)
   {
      for (int i = 0; i < obstacleList.size(); i++)
      {
         CollisionShape obstacle = obstacleList.get(i);
         if (obstacle instanceof CollisionBox)
         {
         }
         else if (obstacle instanceof CollisionCylinder)
         {
            YoFramePoint startLocation = new YoFramePoint("Obstacle" + i + "StartLocation", worldFrame, testRegistry);
            
            YoFrameVector cylinderAxis = new YoFrameVector("Obstacle" + i + "", worldFrame, testRegistry);
            YoGraphicCylinder cylinderGraphic = new YoGraphicCylinder("ObstacleGraphic" + i, startLocation, cylinderAxis, YoAppearance.Red());
            graphicsRegistry.registerYoGraphic("Obstacle" + i, cylinderGraphic);
         }
         else if (obstacle instanceof CollisionSphere)
         {
//            YoFramePoint origin = new YoFramePoint("Obstacle" + i, worldFrame, testRegistry);
//            YoGraphicEllipsoid sphereShape = new YoGraphicEllipsoid("Obstacle" + i, , orientation, appearance, radii)
//            graphicsRegistry.registerYoGraphic("Obstacle" + i, cylinderGraphic);

         }
      }
   }

   private double getBoundedRandomValue(int minBound, int maxBound)
   {
      return Math.random() * (maxBound - minBound);
   }

   @Test
   public void testIK()
   {

   }
}
