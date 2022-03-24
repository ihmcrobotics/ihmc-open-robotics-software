package us.ihmc.valkyrie.controllerAPI;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;

public class HeavyBallOnTableEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D terrain;

   private final double ballRadius = 0.25;
   private final double ballMass = 25.0;
   private final Robot ballRobot;

   public HeavyBallOnTableEnvironment()
   {
      terrain = DefaultCommonAvatarEnvironment.setUpGround("Ground");

      double centerX = 0.6;
      double centerY = 0.0;
      double halfSize = 0.6 / 2.0;
      double edgeThick = 0.05;
      double edgeHeight = 0.10;

      double xStart = centerX - halfSize;
      double yStart = centerY - halfSize;
      double xEnd = centerX + halfSize;
      double yEnd = centerY + halfSize;

      terrain.addBox(xStart, yStart, xEnd, yEnd, 0.8, YoAppearance.Brown());
      terrain.addBox(xStart, yStart, xStart + edgeThick, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());
      terrain.addBox(xEnd - edgeThick, yStart, xEnd, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());

      terrain.addBox(xStart + edgeThick, yStart, xEnd - edgeThick, yStart + edgeThick, 0.8 + edgeHeight, YoAppearance.Brown());
      terrain.addBox(xStart + edgeThick, yEnd - edgeThick, xEnd - edgeThick, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());

      RobotDefinition ballRobotDefinition = createBallRobotDefinition(centerX, centerY);
      ballRobot = new Robot(ballRobotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
   }

   public RobotDefinition createBallRobotDefinition(double centerX, double centerY)
   {
      RobotDefinition ballRobotDefinition = new RobotDefinition("DatBall");
      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("ball");
      RigidBodyDefinition ballBody = new RigidBodyDefinition("ball");
      ballBody.setMass(ballMass);
      ballBody.setMomentOfInertia(MomentOfInertiaFactory.solidSphere(ballMass, ballRadius));
      GeometryDefinition sphereGeometryDefinition = new Sphere3DDefinition(ballRadius);
      ballBody.addCollisionShapeDefinition(new CollisionShapeDefinition(sphereGeometryDefinition));
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Brown());
      materialDefinition.setSpecularColor(ColorDefinitions.Black());
      ballBody.addVisualDefinition(new VisualDefinition(sphereGeometryDefinition, materialDefinition));
      rootJoint.setInitialJointState(new SixDoFJointState(new Quaternion(), new Point3D(centerX, centerY, 0.8 + ballRadius + 0.1)));

      ballRobotDefinition.setRootBodyDefinition(rootBody);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(ballBody);

      return ballRobotDefinition;
   }

   public Robot getBallRobot()
   {
      return ballRobot;
   }

   public Point3DReadOnly getBallRobotPosition()
   {
      SixDoFJointReadOnly rootJoint = (SixDoFJointReadOnly) ballRobot.getRootBody().getChildrenJoints().get(0);
      return rootJoint.getJointPose().getPosition();
   }

   public double getBallRadius()
   {
      return ballRadius;
   }

   @Override
   public CombinedTerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
