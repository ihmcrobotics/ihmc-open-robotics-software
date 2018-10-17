package us.ihmc.valkyrie.simulation;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   private DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testWalkingBetweenBollardsAStarPlanner()
   {
      super.testWalkingBetweenBollardsAStarPlanner();
   }

   @Override
   protected CollisionCheckerScript getCollisionChecker(int simTicksPerCollisionCheck)
   {
      return new ValkyrieLegCollisionDetectorScript(simTicksPerCollisionCheck);
   }

   protected class ValkyrieLegCollisionDetectorScript extends CollisionCheckerScript
   {
      private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
      SimpleCollisionShapeFactory shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();

      // TODO find exact values
      private final double thighRadius = 0.18;
      private final double thighLength = 0.22;

      private final double shinRadius = 0.1;
      private final double shinLength = 0.22;

      private final RigidBodyTransform hipJointToThighCenter = new RigidBodyTransform();
      private final RigidBodyTransform kneeJointToShinCenter = new RigidBodyTransform();
      private final RigidBodyTransform tempTransform = new RigidBodyTransform();

      // TODO add in cylinder viz
      public ValkyrieLegCollisionDetectorScript(int simTicksPerCollisionCheck)
      {
         super(simTicksPerCollisionCheck);
         setupCollisionDetector();
      }

      @Override
      protected boolean collisionDetected()
      {
         // TODO
         return false;
      }

      private void setupCollisionDetector()
      {
         shapeFactory.addShape(shapeFactory.createCylinder(thighRadius, thighLength));
         shapeFactory.addShape(shapeFactory.createCylinder(thighRadius, thighLength));

         shapeFactory.addShape(shapeFactory.createCylinder(shinRadius, shinLength));
         shapeFactory.addShape(shapeFactory.createCylinder(shinRadius, shinLength));

         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(), 0.5 * bollardEnvironment.getBollardWidth(), bollardEnvironment.getBollardHeight()));
         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(), 0.5 * bollardEnvironment.getBollardWidth(), bollardEnvironment.getBollardHeight()));

         collisionDetector.getCollisionObjects().get(0).setCollisionMask(0b110000);
         collisionDetector.getCollisionObjects().get(1).setCollisionMask(0b110000);
         collisionDetector.getCollisionObjects().get(2).setCollisionMask(0b110000);
         collisionDetector.getCollisionObjects().get(3).setCollisionMask(0b110000);
         collisionDetector.getCollisionObjects().get(4).setCollisionMask(0b001111);
         collisionDetector.getCollisionObjects().get(5).setCollisionMask(0b001111);

         collisionDetector.getCollisionObjects().get(0).setCollisionGroup(0);
         collisionDetector.getCollisionObjects().get(1).setCollisionGroup(0);
         collisionDetector.getCollisionObjects().get(2).setCollisionGroup(0);
         collisionDetector.getCollisionObjects().get(3).setCollisionGroup(0);
         collisionDetector.getCollisionObjects().get(4).setCollisionGroup(1);
         collisionDetector.getCollisionObjects().get(5).setCollisionGroup(1);
      }

      private void updateShapeTransform()
      {
         // TODO
      }
   }
}
