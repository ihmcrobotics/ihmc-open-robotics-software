package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import static us.ihmc.exampleSimulations.experimentalPhysicsEngine.ExampleExperimentalSimulationTools.newBoxRobot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BridgeOfBoxesSimulation
{
   private static final String BRIDGE_NAME = "Bridge";
   private static final SideDependentList<String> SUPPORT_NAMES = new SideDependentList<>("LeftSupport", "RightSupport");
   private static final boolean SUPPORT_STATIC = false;

   private final ContactParameters contactParameters = new ContactParameters();

   public BridgeOfBoxesSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.0);
      contactParameters.setRestitutionThreshold(0.0);
      contactParameters.setErrorReductionParameter(0.1);

      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 15);

      Vector3D bridgeSize = new Vector3D(3.0, 0.3, 0.2);
      double bridgeMass = 5.0;
      Vector3D supportSize = new Vector3D(0.5, 0.5, 0.5);
      double supportMass = 5.0;
      RobotDescription bridge = newBoxRobot(BRIDGE_NAME, bridgeSize, bridgeMass, 0.8, YoAppearance.AluminumMaterial());

      double bridgeHeight = 1.0;
      MultiBodySystemStateWriter bridgeInitialStateWriter = MultiBodySystemStateWriter.singleJointStateWriter(BRIDGE_NAME, (FloatingJointBasics joint) ->
      {
         joint.getJointPose().getPosition().setZ(bridgeHeight + 0.5 * bridgeSize.getZ() - 5.0e-4);
      });
      RobotCollisionModel bridgeCollisionModel = RobotCollisionModel.singleBodyCollisionModel(BRIDGE_NAME + "Link", body ->
      {
         return new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), bridgeSize.getX(), bridgeSize.getY(), bridgeSize.getZ()));
      });

      YoRegistry registry = new YoRegistry("controllerInputs");

      SideDependentList<YoFramePoseUsingYawPitchRoll> desiredSupportPoses = new SideDependentList<>(robotSide ->
      {
         YoFramePoseUsingYawPitchRoll pose = new YoFramePoseUsingYawPitchRoll(robotSide.getCamelCaseName() + "SupportPose",
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              registry);
         pose.setPitch(robotSide.negateIfLeftSide(Math.toRadians(45.0)));
         pose.getPosition().set(robotSide.negateIfLeftSide(0.3 * bridgeSize.getX()), 0.0, bridgeHeight);
         pose.appendTranslation(0.5 * robotSide.negateIfLeftSide(supportSize.getX()), 0.0, -0.5 * supportSize.getZ());
         return pose;
      });

      SideDependentList<RobotDescription> supportRobotDescriptions = new SideDependentList<>();
      SideDependentList<MultiBodySystemStateWriter> supportInitialStateWriters = new SideDependentList<>();
      SideDependentList<RobotCollisionModel> supportCollisionModels = new SideDependentList<>();
      List<Collidable> environmentCollidables = new ArrayList<>();

      if (SUPPORT_STATIC)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FrameBox3D supportShape = new FrameBox3D(desiredSupportPoses.get(robotSide), supportSize.getX(), supportSize.getY(), supportSize.getZ());
            environmentCollidables.add(new Collidable(null, -1, -1, supportShape));
         }
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String supportName = SUPPORT_NAMES.get(robotSide);
            supportRobotDescriptions.put(robotSide, newBoxRobot(supportName, supportSize, supportMass, 0.8, YoAppearance.BlackMetalMaterial()));

            Consumer<FloatingJointBasics> jointStateWriter = (FloatingJointBasics joint) -> joint.getJointPose().set(desiredSupportPoses.get(robotSide));
            supportInitialStateWriters.put(robotSide, MultiBodySystemStateWriter.singleJointStateWriter(supportName, jointStateWriter));

            supportCollisionModels.put(robotSide, RobotCollisionModel.singleBodyCollisionModel(supportName + "Link", body ->
            {
               return new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), supportSize.getX(), supportSize.getY(), supportSize.getZ()));
            }));
         }
      }

      double kp = 1000.0;
      double kd = 200.0;
      double frequency = 1.0;
      double amplitude = 0.20;
      SideDependentList<MultiBodySystemStateWriter> supportControllers = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         SpatialVector errorPosition = new SpatialVector();
         SpatialVector errorVelocity = new SpatialVector();
         SpatialVector gravityTerm = new SpatialVector();
         FramePose3D errorPose = new FramePose3D();

         supportControllers.put(robotSide, MultiBodySystemStateWriter.singleJointStateWriter(SUPPORT_NAMES.get(robotSide), (FloatingJointBasics joint) ->
         {
            errorPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredSupportPoses.get(robotSide));

            double offset = amplitude * Math.sin(2.0 * Math.PI * frequency * experimentalSimulation.getPhysicsEngine().getTime());
            errorPose.getPosition().addX(robotSide.negateIfLeftSide(offset));

            errorPose.changeFrame(joint.getFrameAfterJoint());
            errorPosition.setToZero(joint.getFrameAfterJoint());
            errorPosition.getLinearPart().set(errorPose.getPosition());
            errorPose.getOrientation().getRotationVector(errorPosition.getAngularPart());
            errorPosition.scale(kp);

            errorVelocity.setIncludingFrame(joint.getJointTwist());
            errorVelocity.scale(-kd);

            gravityTerm.setToZero(ReferenceFrame.getWorldFrame());
            gravityTerm.setLinearPartZ(9.81 * (supportMass + 0.5 * bridgeMass));
            gravityTerm.changeFrame(joint.getFrameAfterJoint());

            joint.getJointWrench().set(errorPosition);
            joint.getJointWrench().add(errorVelocity);
            joint.getJointWrench().add(gravityTerm);
         }));
      }

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(bridge, bridgeCollisionModel, bridgeInitialStateWriter);
      if (SUPPORT_STATIC)
      {
         experimentalSimulation.addEnvironmentCollidables(environmentCollidables);
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
            experimentalSimulation.addRobot(supportRobotDescriptions.get(robotSide),
                                            supportCollisionModels.get(robotSide),
                                            supportInitialStateWriters.get(robotSide),
                                            supportControllers.get(robotSide));
      }
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      experimentalSimulation.simulate();
      scs.setGroundVisible(false);
      scs.getRootRegistry().addChild(registry);
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.setFastSimulate(true);
      scs.setDT(simDT, 1);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new BridgeOfBoxesSimulation();
   }
}
