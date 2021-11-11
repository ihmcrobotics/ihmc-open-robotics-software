package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.CylinderShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("fast")
@Disabled
public class ValkyrieFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   private static final boolean showCollisionGraphics = true;
   private DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

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

      private final MaterialDefinition collisionAppearance = new MaterialDefinition(ColorDefinitions.SkyBlue().derive(0, 1, 1, 0.6));
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

         if (showCollisionGraphics)
            setupGraphics();
      }

      @Override
      protected boolean collisionDetected()
      {
         // has to be set up first tick cause robot hasn't been set up when constructor is called
         if (firstTick)
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
         tempTransform.getTranslation().setZ(shinZOffset + 0.5 * shinLength);
         shapeFactory.addShape(leftShin, tempTransform, new CylinderShapeDescription<>(shinRadius, shinLength), false, 0b01, 0b10);
         shapeFactory.addShape(rightShin, tempTransform, new CylinderShapeDescription<>(shinRadius, shinLength), false, 0b01, 0b10);

         tempTransform.getTranslation().set(thighXOffset, thighYOffset, thighZOffset + 0.5 * thighLength);
         shapeFactory.addShape(leftThigh, tempTransform, new CylinderShapeDescription<>(thighRadius, thighLength), false, 0b01, 0b10);

         tempTransform.getTranslation().set(thighXOffset, -thighYOffset, thighZOffset + 0.5 * thighLength);
         shapeFactory.addShape(rightThigh, tempTransform, new CylinderShapeDescription<>(thighRadius, thighLength), false, 0b01, 0b10);

         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(),
                                                      0.5 * bollardEnvironment.getBollardWidth(),
                                                      bollardEnvironment.getBollardHeight()));
         shapeFactory.addShape(shapeFactory.createBox(0.5 * bollardEnvironment.getBollardWidth(),
                                                      0.5 * bollardEnvironment.getBollardWidth(),
                                                      bollardEnvironment.getBollardHeight()));

         tempTransform.setIdentity();
         tempTransform.getTranslation().set(0.0, 0.5 * BOLLARD_DISTANCE, 0.0);
         collisionDetector.getCollisionObjects().get(4).setTransformToWorld(tempTransform);

         tempTransform.getTranslation().set(0.0, -0.5 * BOLLARD_DISTANCE, 0.0);
         collisionDetector.getCollisionObjects().get(5).setTransformToWorld(tempTransform);

         collisionDetector.getCollisionObjects().get(4).setCollisionGroup(0b10);
         collisionDetector.getCollisionObjects().get(5).setCollisionGroup(0b10);

         collisionDetector.getCollisionObjects().get(4).setCollisionMask(0b01);
         collisionDetector.getCollisionObjects().get(5).setCollisionMask(0b01);
      }

      private void setupGraphics()
      {
         RobotDefinition robotDescription = getRobotModel().getRobotDefinition();

         JointDefinition leftKneeJoint = robotDescription.getJointDefinition(leftKneeJointName);
         JointDefinition rightKneeJoint = robotDescription.getJointDefinition(rightKneeJointName);
         JointDefinition leftThighJoint = robotDescription.getJointDefinition(leftHipJointName);
         JointDefinition rightThighJoint = robotDescription.getJointDefinition(rightHipJointName);

         RigidBodyDefinition leftKneeSuccessor = leftKneeJoint.getSuccessor();
         RigidBodyDefinition rightKneeSuccessor = rightKneeJoint.getSuccessor();
         RigidBodyDefinition leftThighSuccessor = leftThighJoint.getSuccessor();
         RigidBodyDefinition rightThighSuccessor = rightThighJoint.getSuccessor();

         leftKneeSuccessor.addVisualDefinition(new VisualDefinition(new Point3D(0, 0, shinZOffset),
                                                                    new Cylinder3DDefinition(shinLength, shinRadius, false),
                                                                    collisionAppearance));
         rightKneeSuccessor.addVisualDefinition(new VisualDefinition(new Point3D(0, 0, shinZOffset),
                                                                     new Cylinder3DDefinition(shinLength, shinRadius, false),
                                                                     collisionAppearance));

         leftThighSuccessor.addVisualDefinition(new VisualDefinition(new Point3D(thighXOffset, thighYOffset, thighZOffset),
                                                                     new Cylinder3DDefinition(thighLength, thighRadius, false),
                                                                     collisionAppearance));
         rightThighSuccessor.addVisualDefinition(new VisualDefinition(new Point3D(thighXOffset, -thighYOffset, thighZOffset),
                                                                      new Cylinder3DDefinition(thighLength, thighRadius, false),
                                                                      collisionAppearance));
      }
   }
}
