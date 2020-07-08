package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import java.util.Collections;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;

public class SpinningCoinExperimentalSimulation
{
   private static final String SPINNING_COIN = "SpinningCoin";
   private final ContactParameters contactParameters = new ContactParameters();
   private final double coinWidth = 0.00175; //quarter //0.1    
   private final double coinRadius = 0.01213; //0.5; //         
   private final double coinMass = 0.00567; //1.0; //
   private final double spinningAngularVelocity = 2.0 * Math.PI;

   public SpinningCoinExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.0);
      contactParameters.setRestitutionThreshold(0.0);
      contactParameters.setErrorReductionParameter(1.0e-3);
      
      RobotDescription robotDescription = new RobotDescription(SPINNING_COIN);

      FloatingJointDescription floatingJointDescription = new FloatingJointDescription("root");
      floatingJointDescription.getOffsetFromParentJoint(new Vector3D(0.0, 0.0, 0.0));

      LinkDescription linkDescription = new LinkDescription("coin");
      linkDescription.setMassAndRadiiOfGyration(coinMass, coinRadius / 2.0, coinRadius / 2.0, coinWidth / 2.0);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -coinWidth / 2.0);
      linkGraphics.addCylinder(coinWidth, coinRadius, YoAppearance.Purple());
      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, coinWidth / 2.0);
      linkGraphics.addCube(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, YoAppearance.AliceBlue());
      linkGraphics.translate(0.0, 0.0, -coinWidth - coinWidth / 4.0);
      linkGraphics.addCube(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, YoAppearance.Gold());
      linkDescription.setLinkGraphics(linkGraphics);

      floatingJointDescription.setLink(linkDescription);
      robotDescription.addRootJoint(floatingJointDescription);

      RobotCollisionModel robotCollisionModel = RobotCollisionModel.singleBodyCollisionModel("coin", coinBody ->
      {
         return new Collidable(coinBody, -1, -1, new FrameCylinder3D(coinBody.getBodyFixedFrame(), coinWidth, coinRadius));
      });

      MultiBodySystemStateWriter robotInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter("root", (FloatingJointBasics joint) ->
      {
         joint.getJointPose().set(0.1, 0.1, coinRadius + 0.04, 0.0, 0.0, 1.2);
         joint.getJointTwist().getAngularPart().set(0.0, spinningAngularVelocity, 0.0);
      });

      FrameBox3D groundShape = new FrameBox3D(ReferenceFrame.getWorldFrame(), 1000.0, 1000.0, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(robotDescription, robotCollisionModel, robotInitialStateWriter);
      experimentalSimulation.addEnvironmentCollidables(Collections.singletonList(groundCollidable));
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      ExampleExperimentalSimulationTools.configureSCSToTrackRobotRootJoint(scs, robotDescription);
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(10.0);
   }

   public static void main(String[] args)
   {
      new SpinningCoinExperimentalSimulation();
   }
}
