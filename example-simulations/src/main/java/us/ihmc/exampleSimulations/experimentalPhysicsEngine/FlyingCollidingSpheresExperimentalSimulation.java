package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import static us.ihmc.exampleSimulations.experimentalPhysicsEngine.ExampleExperimentalSimulationTools.newSphereRobot;

import us.ihmc.euclid.referenceFrame.FrameSphere3D;
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

public class FlyingCollidingSpheresExperimentalSimulation
{
   private final ContactParameters contactParameters = new ContactParameters();

   public FlyingCollidingSpheresExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(1.0);

      double radius1 = 0.2;
      double mass1 = 1.0;
      double radiusOfGyrationPercent1 = 1.0;
      AppearanceDefinition appearance1 = YoAppearance.DarkGreen();
      AppearanceDefinition stripesAppearance1 = YoAppearance.LightGreen();
      RobotDescription sphereRobot1 = newSphereRobot("sphere1", radius1, mass1, radiusOfGyrationPercent1, appearance1, true, stripesAppearance1);

      double radius2 = 0.2;
      double mass2 = 1.0;
      double radiusOfGyrationPercent2 = 1.0;
      AppearanceDefinition appearance2 = YoAppearance.DarkRed();
      AppearanceDefinition stripesAppearance2 = YoAppearance.LightSteelBlue();
      RobotDescription sphereRobot2 = newSphereRobot("sphere2", radius2, mass2, radiusOfGyrationPercent2, appearance2, true, stripesAppearance2);

      RobotCollisionModel collisionModel1 = RobotCollisionModel.singleBodyCollisionModel("sphere1Link", body ->
      {
         return new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), radius1));
      });
      RobotCollisionModel collisionModel2 = RobotCollisionModel.singleBodyCollisionModel("sphere2Link", body ->
      {
         return new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), radius1));
      });

      MultiBodySystemStateWriter sphereInitialState1 = MultiBodySystemStateWriter.singleJointStateWriter("sphere1", (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().set(-1.3, 1.0, 0.6);
         joint.getJointTwist().getLinearPart().setX(2.0);
      });
      MultiBodySystemStateWriter sphereInitialState2 = MultiBodySystemStateWriter.singleJointStateWriter("sphere2", (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().set(+0.2, 1.0, 0.4);
      });

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, 0.0));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(sphereRobot1, collisionModel1, sphereInitialState1);
      experimentalSimulation.addRobot(sphereRobot2, collisionModel2, sphereInitialState2);
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      experimentalSimulation.simulate();
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.setGroundVisible(false);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(5.0);
   }

   public static void main(String[] args)
   {
      new FlyingCollidingSpheresExperimentalSimulation();
   }
}
