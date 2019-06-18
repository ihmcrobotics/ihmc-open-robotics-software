package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.CylinderShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Disabled
public class ValkyrieFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   private static final boolean showCollisionGraphics = true;
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
   @Test
   public void testWalkingOnFlatGround()
   {
      super.testWalkingOnFlatGround();
   }

   @Override
   @Test
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

      private final double thighRadius = 0.12;
      private final double thighLength = 0.45;
      private final double thighXOffset = 0.04;
      private final double thighYOffset = 0.1;
      private final double thighZOffset = -0.45;

      private final double shinRadius = 0.13;
      private final double shinLength = 0.25;
      private final double shinZOffset = -0.3;

      private final AppearanceDefinition collisionAppearance = YoAppearance.Glass(0.4);
      private final RigidBodyTransform tempTransform = new RigidBodyTransform();
      private final CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

      private final String leftKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      private final String rightKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
      private final String leftHipJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH);
      private final String rightHipJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH);

      private boolean firstTick = true;

      public ValkyrieLegCollisionDetectorScript(int simTicksPerCollisionCheck)
      {
         super(simTicksPerCollisionCheck);

         if(showCollisionGraphics)
            setupGraphics();
      }

      @Override
      protected boolean collisionDetected()
      {
         // has to be set up first tick cause robot hasn't been set up when constructor is called
         if(firstTick)
         {
            setupCollisionDetector();
            firstTick = false;
         }

         collisionDetector.performCollisionDetection(collisionDetectionResult);
         return collisionDetectionResult.getNumberOfCollisions() > 0;
      }

      private void setupCollisionDetector()
      {
         HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
         Link leftShin = robot.getJoint(leftKneeJointName).getLink();
         Link rightShin = robot.getJoint(rightKneeJointName).getLink();
         Link leftThigh = robot.getJoint(leftHipJointName).getLink();
         Link rightThigh = robot.getJoint(rightHipJointName).getLink();

         tempTransform.setIdentity();
         tempTransform.setTranslationZ(shinZOffset + 0.5 * shinLength);
         shapeFactory.addShape(leftShin, tempTransform, new CylinderShapeDescription<>(shinRadius, shinLength), false, 0b01, 0b10);
         shapeFactory.addShape(rightShin, tempTransform, new CylinderShapeDescription<>(shinRadius, shinLength), false, 0b01, 0b10);

         tempTransform.setTranslation(thighXOffset, thighYOffset, thighZOffset + 0.5 * thighLength);
         shapeFactory.addShape(leftThigh, tempTransform, new CylinderShapeDescription<>(thighRadius, thighLength), false, 0b01, 0b10);

         tempTransform.setTranslation(thighXOffset, - thighYOffset, thighZOffset + 0.5 * thighLength);
         shapeFactory.addShape(rightThigh, tempTransform, new CylinderShapeDescription<>(thighRadius, thighLength), false, 0b01, 0b10);

         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(), 0.5 * bollardEnvironment.getBollardWidth(), bollardEnvironment.getBollardHeight()));
         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(), 0.5 * bollardEnvironment.getBollardWidth(), bollardEnvironment.getBollardHeight()));

         tempTransform.setIdentity();
         tempTransform.setTranslation(0.0, 0.5 * BOLLARD_DISTANCE, 0.0);
         collisionDetector.getCollisionObjects().get(4).setTransformToWorld(tempTransform);

         tempTransform.setTranslation(0.0, - 0.5 * BOLLARD_DISTANCE, 0.0);
         collisionDetector.getCollisionObjects().get(5).setTransformToWorld(tempTransform);

         collisionDetector.getCollisionObjects().get(4).setCollisionGroup(0b10);
         collisionDetector.getCollisionObjects().get(5).setCollisionGroup(0b10);

         collisionDetector.getCollisionObjects().get(4).setCollisionMask(0b01);
         collisionDetector.getCollisionObjects().get(5).setCollisionMask(0b01);
      }

      private void setupGraphics()
      {
         RobotDescription robotDescription = getRobotModel().getRobotDescription();

         JointDescription leftKneeJoint = robotDescription.getJointDescription(leftKneeJointName);
         JointDescription rightKneeJoint = robotDescription.getJointDescription(rightKneeJointName);
         JointDescription leftThighJoint = robotDescription.getJointDescription(leftHipJointName);
         JointDescription rightThighJoint = robotDescription.getJointDescription(rightHipJointName);

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
         leftThighGraphics.translate(thighXOffset, thighYOffset, thighZOffset);
         leftThighGraphics.addCylinder(thighLength, thighRadius, collisionAppearance);

         rightThighGraphics.identity();
         rightThighGraphics.translate(thighXOffset, -thighYOffset, thighZOffset);
         rightThighGraphics.addCylinder(thighLength, thighRadius, collisionAppearance);
      }
   }
}
