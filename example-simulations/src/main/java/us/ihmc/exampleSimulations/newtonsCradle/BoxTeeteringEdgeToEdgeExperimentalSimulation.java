package us.ihmc.exampleSimulations.newtonsCradle;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
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

public class BoxTeeteringEdgeToEdgeExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ContactParameters contactParameters = new ContactParameters(0.7, 0.0, 0.001, 1.0);

   public BoxTeeteringEdgeToEdgeExperimentalSimulation()
   {
      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      RobotDescription boxRobot = createASingleBoxRobot("box", boxXLength, boxYWidth, boxZHeight, boxMass, boxRadiusOfGyrationPercent, YoAppearance.DarkCyan());

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
      Graphics3DObject groundGraphics = new Graphics3DObject();
      groundGraphics.translate(0, 0, -0.1);
      groundGraphics.addCube(groundLength, groundWidth, 0.1, YoAppearance.DarkKhaki());

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(groundGraphics);
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   private static RobotDescription createASingleBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent,
                                                         AppearanceDefinition appearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription link = new LinkDescription(name + "Link");
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * sizeX, radiusOfGyrationPercent * sizeY, radiusOfGyrationPercent * sizeZ);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -sizeZ / 2.0);
      linkGraphics.addCube(sizeX, sizeY, sizeZ, appearance);
      link.setLinkGraphics(linkGraphics);

      rootJoint.setLink(link);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

   public static void main(String[] args)
   {
      new BoxTeeteringEdgeToEdgeExperimentalSimulation();
   }
}
