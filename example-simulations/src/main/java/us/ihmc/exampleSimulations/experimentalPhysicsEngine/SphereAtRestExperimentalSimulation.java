package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
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

public class SphereAtRestExperimentalSimulation
{
   private final ContactParameters contactParameters = new ContactParameters();

   public SphereAtRestExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setErrorReductionParameter(0.01);

      double sphereRadius = 0.5;
      double sphereMass = 1.0;

      RobotDescription sphereRobot = ExampleExperimentalSimulationTools.newSphereRobot("Sphere",
                                                                                       sphereRadius,
                                                                                       sphereMass,
                                                                                       0.8,
                                                                                       YoAppearance.AluminumMaterial(),
                                                                                       true,
                                                                                       YoAppearance.Gold());
      MultiBodySystemStateWriter sphereInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter("Sphere", (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().setZ(sphereRadius);
//         joint.getJointPose().getPosition().set(0.0, 0.0, 1.0);
//         joint.getJointTwist().getLinearPart().set(10.0, 0.0, 10.0);
//         joint.getJointTwist().getAngularPart().set(0.0, 0.0, 200.0);
      });
      RobotCollisionModel sphereCollisionModel = RobotCollisionModel.singleBodyCollisionModel("SphereLink", body ->
      {
         return new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), sphereRadius));
      });

      FrameBox3D groundShape = new FrameBox3D(ReferenceFrame.getWorldFrame(), 100.0, 100.0, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 15);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -10.0));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(sphereRobot, sphereCollisionModel, sphereInitialStateWriter);
      experimentalSimulation.addEnvironmentCollidable(groundCollidable);
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      ExampleExperimentalSimulationTools.configureSCSToTrackRobotRootJoint(scs, sphereRobot);
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new SphereAtRestExperimentalSimulation();
   }
}
