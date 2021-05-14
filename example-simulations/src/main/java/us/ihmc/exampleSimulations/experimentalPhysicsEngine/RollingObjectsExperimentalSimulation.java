package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;

public class RollingObjectsExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String BALL_NAME = "ball";
   private static final String CAPSULE_NAME = "capsule";
   private static final String CYLINDER_NAME = "cylinder";
   private static final String BALL_BODY_NAME = BALL_NAME + "Link";
   private static final String CAPSULE_BODY_NAME = CAPSULE_NAME + "Link";
   private static final String CYLINDER_BODY_NAME = CYLINDER_NAME + "Link";

   private final ContactParameters contactParameters = new ContactParameters();

   public RollingObjectsExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      double ballRadius = 0.2;
      double ballMass = 1.0;
      double ballRadiusOfGyrationPercent = 1.0;

      double cylinderRadius = 0.2;
      double cylinderHeight = 0.5;
      double cylinderMass = 1.0;
      double cylinderRadiusOfGyrationPercent = 1.0;

      double capsuleRadius = 0.2;
      double capsuleHeight = 0.1 + 2.0 * capsuleRadius;
      double capsuleMass = 1.0;
      double capsuleRadiusOfGyrationPercent = 1.0;

      double initialVelocity = 1.0;

      AppearanceDefinition appearance = YoAppearance.DarkCyan();
      boolean addStripes = true;
      AppearanceDefinition stripesAppearance = YoAppearance.Gold();

      RobotDescription ballRobot = ExampleExperimentalSimulationTools.newSphereRobot(BALL_NAME,
                                                                                     ballRadius,
                                                                                     ballMass,
                                                                                     ballRadiusOfGyrationPercent,
                                                                                     appearance,
                                                                                     addStripes,
                                                                                     stripesAppearance);
      RobotDescription cylinderRobot = ExampleExperimentalSimulationTools.newCylinderRobot(CYLINDER_NAME,
                                                                                           cylinderRadius,
                                                                                           cylinderHeight,
                                                                                           cylinderMass,
                                                                                           cylinderRadiusOfGyrationPercent,
                                                                                           appearance,
                                                                                           addStripes,
                                                                                           stripesAppearance);
      RobotDescription capsuleRobot = ExampleExperimentalSimulationTools.newCapsuleRobot(CAPSULE_NAME,
                                                                                         capsuleRadius,
                                                                                         capsuleHeight,
                                                                                         capsuleMass,
                                                                                         capsuleRadiusOfGyrationPercent,
                                                                                         appearance,
                                                                                         addStripes,
                                                                                         stripesAppearance);

      MultiBodySystemStateWriter ballInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter(BALL_NAME, (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().set(-1.0, -2.0 * ballRadius - cylinderHeight, ballRadius * 1.02);
         joint.getJointTwist().getLinearPart().setMatchingFrame(new FrameVector3D(worldFrame, initialVelocity, 0.0, 0.0));
      });
      MultiBodySystemStateWriter cylinderInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter(CYLINDER_NAME, (FloatingJointBasics joint) ->
      {
         joint.getJointPose().set(-1.0, 0.0, cylinderRadius * 1.02, 0.0, 0.0, Math.PI / 2.0);
         joint.getJointTwist().getLinearPart().setMatchingFrame(new FrameVector3D(worldFrame, initialVelocity, 0.0, 0.0));
      });
      MultiBodySystemStateWriter capsuleInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter(CAPSULE_NAME, (FloatingJointBasics joint) ->
      {
         joint.getJointPose().set(-1.0, cylinderHeight + capsuleHeight + capsuleRadius, capsuleRadius * 1.02, 0.0, 0.0, Math.PI / 2.0);
         joint.getJointTwist().getLinearPart().setMatchingFrame(new FrameVector3D(worldFrame, initialVelocity, 0.0, 0.0));
      });

      RobotCollisionModel ballCollision = RobotCollisionModel.singleBodyCollisionModel(BALL_BODY_NAME, body ->
      {
         return new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), ballRadius));
      });
      RobotCollisionModel cylinderCollision = RobotCollisionModel.singleBodyCollisionModel(CYLINDER_BODY_NAME, body ->
      {
         return new Collidable(body, -1, -1, new FrameCylinder3D(body.getBodyFixedFrame(), cylinderHeight, cylinderRadius));
      });
      RobotCollisionModel capsuleCollision = RobotCollisionModel.singleBodyCollisionModel(CAPSULE_BODY_NAME, body ->
      {
         return new Collidable(body, -1, -1, new FrameCapsule3D(body.getBodyFixedFrame(), cylinderHeight, cylinderRadius));
      });

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(ballRobot, ballCollision, ballInitialStateWriter);
      experimentalSimulation.addRobot(cylinderRobot, cylinderCollision, cylinderInitialStateWriter);
      experimentalSimulation.addRobot(capsuleRobot, capsuleCollision, capsuleInitialStateWriter);

      FrameBox3D groundShape = new FrameBox3D(worldFrame, 1000.0, 1000.0, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);
      experimentalSimulation.addEnvironmentCollidable(groundCollidable);

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new RollingObjectsExperimentalSimulation();
   }
}
