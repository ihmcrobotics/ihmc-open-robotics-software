package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
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

public class BoxTeeteringEdgeToEdgeExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ContactParameters contactParameters = new ContactParameters();

   public BoxTeeteringEdgeToEdgeExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      RobotDescription boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box",
                                                                                 boxXLength,
                                                                                 boxYWidth,
                                                                                 boxZHeight,
                                                                                 boxMass,
                                                                                 boxRadiusOfGyrationPercent,
                                                                                 YoAppearance.DarkCyan());

      MultiBodySystemStateWriter boxInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter("box", (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().set(0.0, groundWidth / 2.0 - 0.002, boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll)));
         joint.getJointPose().getOrientation().setToRollOrientation(initialBoxRoll);
         joint.getJointTwist().getLinearPart().setMatchingFrame(new FrameVector3D(worldFrame, initialVelocity, 0, 0));
      });

      RobotCollisionModel boxCollision = RobotCollisionModel.singleBodyCollisionModel("boxLink", body ->
      {
         return new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), boxXLength, boxYWidth, boxZHeight));
      });

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(boxRobot, boxCollision, boxInitialStateWriter);

      FrameBox3D groundShape = new FrameBox3D(worldFrame, groundLength, groundWidth, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);
      experimentalSimulation.addEnvironmentCollidable(groundCollidable);

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(ExampleExperimentalSimulationTools.toGraphics3DObject(groundShape, worldFrame, YoAppearance.DarkKhaki()));
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new BoxTeeteringEdgeToEdgeExperimentalSimulation();
   }
}
