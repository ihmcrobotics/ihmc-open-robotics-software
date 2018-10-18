package us.ihmc.valkyrie.simulation;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
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

      private final double thighRadius = 0.1;
      private final double thighLength = 0.5;
      private final double thighYOffset = 0.1;
      private final double thighZOffset = -0.5;

      private final double shinRadius = 0.13;
      private final double shinLength = 0.25;
      private final double shinZOffset = -0.3;

      private final AppearanceDefinition collisionAppearance = YoAppearance.Glass(0.4);

      public ValkyrieLegCollisionDetectorScript(int simTicksPerCollisionCheck)
      {
         super(simTicksPerCollisionCheck);
         setupGraphics();
         setupCollisionDetector();
      }

      @Override
      protected boolean collisionDetected()
      {
         updateShapeTransform();
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

      private void setupGraphics()
      {
         RobotDescription robotDescription = getRobotModel().getRobotDescription();

         String leftKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH);
         String rightKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
         String leftThighJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH);
         String rightThighJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH);

         JointDescription leftKneeJoint = robotDescription.getJointDescription(leftKneeJointName);
         JointDescription rightKneeJoint = robotDescription.getJointDescription(rightKneeJointName);
         JointDescription leftThighJoint = robotDescription.getJointDescription(leftThighJointName);
         JointDescription rightThighJoint = robotDescription.getJointDescription(rightThighJointName);

         Graphics3DObject leftKneeGraphics = leftKneeJoint.getLink().getLinkGraphics();
         Graphics3DObject rightKneeGraphics = rightKneeJoint.getLink().getLinkGraphics();
         Graphics3DObject leftThighGraphics = leftThighJoint.getLink().getLinkGraphics();
         Graphics3DObject rightThighGraphics = rightThighJoint.getLink().getLinkGraphics();

         leftKneeGraphics.identity();
         leftKneeGraphics.translate(0.0, 0.0, shinZOffset);
         leftKneeGraphics.addCylinder(shinLength, shinRadius, collisionAppearance);

         rightKneeGraphics.identity();
         rightKneeGraphics.translate(0.0, 0.0, shinZOffset);
         rightKneeGraphics.addCylinder(shinLength, shinRadius, collisionAppearance);

         leftThighGraphics.identity();
         leftThighGraphics.translate(0.0, thighYOffset, thighZOffset);
         leftThighGraphics.addCylinder(thighLength, thighRadius, collisionAppearance);

         rightThighGraphics.identity();
         rightThighGraphics.translate(0.0, -thighYOffset, thighZOffset);
         rightThighGraphics.addCylinder(thighLength, thighRadius, collisionAppearance);
      }
   }
}
