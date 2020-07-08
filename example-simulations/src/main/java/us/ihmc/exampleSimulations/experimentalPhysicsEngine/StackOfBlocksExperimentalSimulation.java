package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
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

public class StackOfBlocksExperimentalSimulation
{
   private final ContactParameters contactParameters = new ContactParameters();

   public StackOfBlocksExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.3);
      contactParameters.setRestitutionThreshold(0.15);
      contactParameters.setErrorReductionParameter(0.01);

      int numberOfBlocks = 6;
      Random random = new Random(1886L);

      List<RobotDescription> robotDescriptions = new ArrayList<>();
      List<RobotCollisionModel> robotCollisionModels = new ArrayList<>();
      List<MultiBodySystemStateWriter> robotInitialStateWriters = new ArrayList<>();

      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      double mass = 0.2;

      for (int i = 0; i < numberOfBlocks; i++)
      {
         AppearanceDefinition appearance = YoAppearance.randomColor(random);
         robotDescriptions.add(ExampleExperimentalSimulationTools.newBoxRobot("Block" + i, boxSizeX, boxSizeY, boxSizeZ, mass, 0.5, appearance));

         robotCollisionModels.add(RobotCollisionModel.singleBodyCollisionModel("Block" + i + "Link", body ->
         {
            return new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), boxSizeX, boxSizeY, boxSizeZ));
         }));

         double x = 0.0;
         double y = 0.0;
         double z = boxSizeZ * 1.05 * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
         double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

         robotInitialStateWriters.add(MultiBodySystemStateWriter.singleJointStateWriter("Block" + i, (FloatingJointBasics joint) ->
         {
            joint.getJointPose().set(x, y, z, yaw, pitch, roll);
         }));
      }

      FrameBox3D groundShape = new FrameBox3D(ReferenceFrame.getWorldFrame(), 1000.0, 1000.0, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      for (int i = 0; i < numberOfBlocks; i++)
      {
         experimentalSimulation.addRobot(robotDescriptions.get(i), robotCollisionModels.get(i), robotInitialStateWriters.get(i));
      }
      experimentalSimulation.addEnvironmentCollidables(Collections.singletonList(groundCollidable));
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      experimentalSimulation.simulate();
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(5.0);
   }

   public static void main(String[] args)
   {
      new StackOfBlocksExperimentalSimulation();
   }
}
