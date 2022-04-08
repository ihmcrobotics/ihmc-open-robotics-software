package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHolder;
import us.ihmc.scs2.simulation.collision.CollisionListResult;
import us.ihmc.scs2.simulation.physicsEngine.SimpleCollisionDetection;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
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

   private static final ReferenceFrame InertialFrame = SimulationSession.DEFAULT_INERTIAL_FRAME;

   protected class ValkyrieLegCollisionDetectorScript extends CollisionCheckerScript
   {
      private SimpleCollisionDetection collisionDetector = new SimpleCollisionDetection(InertialFrame);

      private final double thighRadius = 0.12;
      private final double thighLength = 0.45;
      private final double thighXOffset = 0.04;
      private final double thighYOffset = 0.1;
      private final double thighZOffset = -0.45;

      private final double shinRadius = 0.13;
      private final double shinLength = 0.25;
      private final double shinZOffset = -0.3;

      private final MaterialDefinition collisionAppearance = new MaterialDefinition(ColorDefinitions.SkyBlue().derive(0, 1, 1, 0.6));

      private final String leftKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      private final String rightKneeJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
      private final String leftHipJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH);
      private final String rightHipJointName = getRobotModel().getJointMap().getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH);

      private boolean firstTick = true;

      private CollidableHolder dynamicCollidableHolder;
      private CollidableHolder staticCollidableHolder;

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

         CollisionListResult result = collisionDetector.evaluationCollisions(Collections.singletonList(dynamicCollidableHolder),
                                                                             staticCollidableHolder,
                                                                             simulationTestHelper.getSimulationDT());
         return result.size() > 0;
      }

      private void setupCollisionDetector()
      {
         Robot robot = simulationTestHelper.getRobot();
         SimRigidBodyBasics leftShin = robot.getJoint(leftKneeJointName).getSuccessor();
         SimRigidBodyBasics rightShin = robot.getJoint(rightKneeJointName).getSuccessor();
         SimRigidBodyBasics leftThigh = robot.getJoint(leftHipJointName).getSuccessor();
         SimRigidBodyBasics rightThigh = robot.getJoint(rightHipJointName).getSuccessor();

         Collidable leftShinCollidable = new Collidable(leftShin,
                                                        0b10,
                                                        0b01,
                                                        new FrameCylinder3D(leftShin.getParentJoint().getFrameAfterJoint(),
                                                                            new Point3D(0.0, 0.0, shinZOffset + 0.5 * shinLength),
                                                                            Axis3D.Z,
                                                                            shinLength,
                                                                            shinRadius));
         Collidable rightShinCollidable = new Collidable(rightShin,
                                                         0b10,
                                                         0b01,
                                                         new FrameCylinder3D(rightShin.getParentJoint().getFrameAfterJoint(),
                                                                             new Point3D(0.0, 0.0, shinZOffset + 0.5 * shinLength),
                                                                             Axis3D.Z,
                                                                             shinLength,
                                                                             shinRadius));
         Collidable leftThighCollidable = new Collidable(leftThigh,
                                                         0b10,
                                                         0b01,
                                                         new FrameCylinder3D(leftThigh.getParentJoint().getFrameAfterJoint(),
                                                                             new Point3D(thighXOffset, thighYOffset, thighZOffset + 0.5 * thighLength),
                                                                             Axis3D.Z,
                                                                             thighLength,
                                                                             thighRadius));
         Collidable rightThighCollidable = new Collidable(rightThigh,
                                                          0b10,
                                                          0b01,
                                                          new FrameCylinder3D(rightThigh.getParentJoint().getFrameAfterJoint(),
                                                                              new Point3D(thighXOffset, -thighYOffset, thighZOffset + 0.5 * thighLength),
                                                                              Axis3D.Z,
                                                                              thighLength,
                                                                              thighRadius));
         List<Collidable> robotCollidables = Arrays.asList(leftShinCollidable, rightShinCollidable, leftThighCollidable, rightThighCollidable);
         dynamicCollidableHolder = () -> robotCollidables;

         List<Collidable> enironmentCollidables = new ArrayList<>();
         enironmentCollidables.add(new Collidable(null,
                                                  0b01,
                                                  0b10,
                                                  new FrameBox3D(new FramePoint3D(InertialFrame, 0.0, 0.5 * BOLLARD_DISTANCE, 0.0),
                                                                 new FrameQuaternion(InertialFrame),
                                                                 0.5 * bollardEnvironment.getBollardWidth(),
                                                                 0.5 * bollardEnvironment.getBollardWidth(),
                                                                 bollardEnvironment.getBollardHeight())));
         enironmentCollidables.add(new Collidable(null,
                                                  0b01,
                                                  0b10,
                                                  new FrameBox3D(new FramePoint3D(InertialFrame, 0.0, -0.5 * BOLLARD_DISTANCE, 0.0),
                                                                 new FrameQuaternion(InertialFrame),
                                                                 0.5 * bollardEnvironment.getBollardWidth(),
                                                                 0.5 * bollardEnvironment.getBollardWidth(),
                                                                 bollardEnvironment.getBollardHeight())));
         staticCollidableHolder = () -> enironmentCollidables;
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
