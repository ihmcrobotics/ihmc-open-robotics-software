package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter.MapBasedJointStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;

public class ConnectedShapesExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ContactParameters contactParameters = new ContactParameters();

   public ConnectedShapesExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);
      contactParameters.setCoefficientOfRestitution(0.5);
      contactParameters.setRestitutionThreshold(0.15);

      Vector3D boxSize1 = new Vector3D(0.5, 0.3, 0.3);
      double boxMass1 = 1.0;
      double radiusOfGyrationPercent = 0.8;
      AppearanceDefinition boxApp1 = YoAppearance.LightSeaGreen();

      Vector3D boxSize2 = new Vector3D(0.3, 0.3, 0.3);
      double boxMass2 = 1.0;
      AppearanceDefinition boxApp2 = YoAppearance.Teal();

      Vector3D connectionOffset = new Vector3D(0.9, 0.0, 0.0);

      RobotDescription robotDescription = new RobotDescription("ConnectedShapes");
      FloatingJointDescription rootJointDescription = new FloatingJointDescription("root");
      LinkDescription link1 = ExampleExperimentalSimulationTools.newBoxLink("box1", boxSize1, boxMass1, radiusOfGyrationPercent, boxApp1);
      rootJointDescription.setLink(link1);

      PinJointDescription pinJoint = new PinJointDescription("pin", new Vector3D(), Axis3D.Y);
      LinkDescription link2 = ExampleExperimentalSimulationTools.newBoxLink("box2", boxSize2, boxMass2, radiusOfGyrationPercent, boxApp2);
      link2.setCenterOfMassOffset(connectionOffset);
      LinkGraphicsDescription linkGraphics2 = new LinkGraphicsDescription();
      linkGraphics2.rotate(0.5 * Math.PI, Axis3D.Y);
      linkGraphics2.addCylinder(connectionOffset.getX(), 0.02, YoAppearance.AluminumMaterial());
      linkGraphics2.combine(link2.getLinkGraphics(), connectionOffset);
      link2.setLinkGraphics(linkGraphics2);
      pinJoint.setLink(link2);

      robotDescription.addRootJoint(rootJointDescription);
      rootJointDescription.addJoint(pinJoint);

      MapBasedJointStateWriter initialStateWriter = new MapBasedJointStateWriter()
      {
         @Override
         public boolean write()
         {
            FloatingJointBasics rootJoint = getJoint("root");
            rootJoint.getJointPose().set(0.0, 0.0, boxSize1.getZ(), 0.0, 0.0, 0.0);
            RevoluteJointBasics pinJoint = getJoint("pin");
            pinJoint.setQ(0.0);
            pinJoint.setTau(2.0);
            return true;
         }
      };

      RobotCollisionModel collisionModel = new RobotCollisionModel()
      {
         @Override
         public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
         {
            List<Collidable> collidables = new ArrayList<>();
            RigidBodyBasics boxBody1 = RobotCollisionModel.findRigidBody("box1", multiBodySystem);
            RigidBodyBasics boxBody2 = RobotCollisionModel.findRigidBody("box2", multiBodySystem);
            collidables.add(new Collidable(boxBody1, -1, -1, new FrameBox3D(boxBody1.getBodyFixedFrame(), boxSize1)));
            collidables.add(new Collidable(boxBody2, -1, -1, new FrameBox3D(boxBody2.getBodyFixedFrame(), boxSize2)));
            return collidables;
         }
      };

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(robotDescription, collisionModel, initialStateWriter);

      FrameBox3D groundShape = new FrameBox3D(worldFrame, 5.0, 5.0, 0.1);
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
      //      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new ConnectedShapesExperimentalSimulation();
   }
}
