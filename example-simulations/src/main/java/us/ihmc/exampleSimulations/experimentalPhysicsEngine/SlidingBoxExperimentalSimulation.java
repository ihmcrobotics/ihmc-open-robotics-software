package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
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

public class SlidingBoxExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ContactParameters contactParameters = new ContactParameters();

   public SlidingBoxExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      Vector3D boxSize = new Vector3D(0.4, 0.4, 0.4);

      double groundPitch = Math.toRadians(34.0);

      RobotDescription boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box", boxSize, 150.0, 0.8, YoAppearance.DarkCyan());

      MultiBodySystemStateWriter boxInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter("box", (FloatingJointBasics joint) ->
      {
         Pose3DBasics jointPose = joint.getJointPose();
         jointPose.getOrientation().setToPitchOrientation(groundPitch);
         jointPose.appendTranslation(0.0, 0.0, 0.6 * boxSize.getZ());
         joint.getJointTwist().getAngularPart().set(0, 0, 0.0);
         joint.getJointTwist().getLinearPart().setMatchingFrame(new FrameVector3D(worldFrame, 0, 0, 0));
      });

      RobotCollisionModel boxCollision = RobotCollisionModel.singleBodyCollisionModel("boxLink", body ->
      {
         return new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), boxSize.getX(), boxSize.getY(), boxSize.getZ()));
      });

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(boxRobot, boxCollision, boxInitialStateWriter);

      FrameBox3D groundShape = new FrameBox3D(worldFrame, 100.0, 100.0, 0.1);
      groundShape.getOrientation().setToPitchOrientation(groundPitch);
      groundShape.getPose().appendTranslation(0.0, 0.0, -0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);
      experimentalSimulation.addEnvironmentCollidable(groundCollidable);

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      ExampleExperimentalSimulationTools.configureSCSToTrackRobotRootJoint(scs, boxRobot);
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
      new SlidingBoxExperimentalSimulation();
   }
}
