package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.geometry.shapes.FrameSTPBox3D;
import us.ihmc.robotics.physics.*;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter.MapBasedJointStateWriter;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimplifiedAnkleExperimentalSimulation
{
   private final String robotName = "ankleRobot";
   private final String rootJointName = "rootJoint";
   private final String anklePitchName = "anklePitch";
   private final String ankleRollName = "ankleRoll";

   private final String comLinkName = "comLink";
   private final String footLinkName = "foot";

   private final ContactParameters contactParameters = new ContactParameters();
   private final Vector3D footSize = new Vector3D(5 * 0.3, 5 * 0.15, 0.08);
   private final Vector3D footOffset = new Vector3D(footSize.getX() / 3.75, 0.0, -0.5 * footSize.getZ());
   private final double footMass = 1.2;
   private final double ankleJointSeparation = 0.0;

   private final double comBallRadius = 0.3;
   private final double legLength = 1.0;
   private final double comBallMass = 120.0;

   private final YoRegistry controllerRegitry = new YoRegistry("controllerRegistry");
   private final YoPDGains ankleGains = new YoPDGains("Ankle", controllerRegitry);

   private final YoDouble desiredPositionAnklePitch = new YoDouble("desiredPositionAnklePitch", controllerRegitry);
   private final YoDouble desiredPositionAnkleRoll = new YoDouble("desiredPositionAnkleRoll", controllerRegitry);

   private final YoDouble feedForwardTorqueAnklePitch = new YoDouble("feedForwardTorqueAnklePitch", controllerRegitry);
   private final YoDouble feedForwardTorqueAnkleRoll = new YoDouble("feedForwardTorqueAnkleRoll", controllerRegitry);

   public SimplifiedAnkleExperimentalSimulation() throws UnreasonableAccelerationException
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);

      RobotDescription robotDescription = createRobotDescription();
      MultiBodySystemStateWriter initialStateWriter = createInitialStateWriter();
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(footLinkName, body ->
      {
         FrameSTPBox3D shape = new FrameSTPBox3D(body.getBodyFixedFrame(), footSize);
         shape.setMargins(1.0e-4, 1.0e-3);
         return new Collidable(body, -1, -1, shape);
      });

      MultiBodySystemStateWriter controller = createController();

      FrameBox3D groundShape = new FrameBox3D(ReferenceFrame.getWorldFrame(), 100.0, 100.0, 0.1);
      groundShape.getPosition().subZ(0.05);
      Collidable groundCollidable = new Collidable(null, -1, -1, groundShape);

      double simDT = 0.001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(robotDescription, collisionModel, initialStateWriter, controller);
      experimentalSimulation.addEnvironmentCollidable(groundCollidable);
      experimentalSimulation.getPhysicsEngine().addRobotPhysicsOutputStateReader(robotName, createPhysicsOutputReader());
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      scs.getRootRegistry().addChild(controllerRegitry);
      experimentalSimulation.simulate();
      ExampleExperimentalSimulationTools.configureSCSToTrackRobotRootJoint(scs, robotDescription);
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
      scs.startOnAThread();
      scs.simulate(15.0);
   }

   public MultiBodySystemStateWriter createController()
   {
      desiredPositionAnklePitch.set(-0.30);
      ankleGains.setKp(500.0);
      ankleGains.setKd(150.0);

      MultiBodySystemStateWriter controller = new MapBasedJointStateWriter()
      {
         FrameVector3D forceWorld = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 9.81 * comBallMass);

         @Override
         public boolean write()
         {
            OneDoFJointBasics anklePitch = getJoint(anklePitchName);
            OneDoFJointBasics ankleRoll = getJoint(ankleRollName);
            MultiBodySystemTools.getRootBody(anklePitch.getPredecessor()).updateFramesRecursively();

            MovingReferenceFrame comFrame = getJoint(rootJointName).getSuccessor().getBodyFixedFrame();
            SpatialForce antiGravityForce = new SpatialForce(comFrame);
            antiGravityForce.getLinearPart().setMatchingFrame(forceWorld);
            antiGravityForce.changeFrame(anklePitch.getFrameBeforeJoint());

            double tau_ff_anklePitch = -anklePitch.getJointAxis().dot(antiGravityForce.getAngularPart());
            feedForwardTorqueAnklePitch.set(tau_ff_anklePitch);
            double q_anklePitch = anklePitch.getQ();
            double qd_anklePitch = anklePitch.getQd();
            double tau_fb_anklePitch = ankleGains.getKp() * (desiredPositionAnklePitch.getValue() - q_anklePitch) - ankleGains.getKd() * qd_anklePitch;

            anklePitch.setTau(tau_fb_anklePitch + tau_ff_anklePitch);

            antiGravityForce.changeFrame(ankleRoll.getFrameBeforeJoint());
            double tau_ff_ankleRoll = -ankleRoll.getJointAxis().dot(antiGravityForce.getAngularPart());
            feedForwardTorqueAnkleRoll.set(tau_ff_ankleRoll);
            double q_ankleRoll = ankleRoll.getQ();
            double qd_ankleRoll = ankleRoll.getQd();
            double tau_fb_ankleRoll = ankleGains.getKp() * (desiredPositionAnkleRoll.getValue() - q_ankleRoll) - ankleGains.getKd() * qd_ankleRoll;

            ankleRoll.setTau(tau_fb_ankleRoll + tau_ff_ankleRoll);
            return true;
         }
      };
      return controller;
   }

   public MultiBodySystemStateReader createPhysicsOutputReader()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      YoFramePoseUsingYawPitchRoll yoFootCoMPose = new YoFramePoseUsingYawPitchRoll("footCoM", worldFrame, controllerRegitry);
      YoFixedFrameSpatialVector yoFootCoMVelocity = new YoFixedFrameSpatialVector("footCoMVelocity", worldFrame, controllerRegitry);

      YoFramePoint3D yoToePosition = new YoFramePoint3D("toePosition", worldFrame, controllerRegitry);
      YoFrameVector3D yoToeLinearVelocity = new YoFrameVector3D("toeLinearVelocity", worldFrame, controllerRegitry);
      SideDependentList<YoFramePoint3D> yoToeCornerPositions = new SideDependentList<>(new YoFramePoint3D("toeLeftPosition", worldFrame, controllerRegitry),
                                                                                       new YoFramePoint3D("toeRightPosition", worldFrame, controllerRegitry));
      SideDependentList<YoFrameVector3D> yoToeCornerLinearVelocities = new SideDependentList<>(new YoFrameVector3D("toeLeftLinearVelocity",
                                                                                                                   worldFrame,
                                                                                                                   controllerRegitry),
                                                                                               new YoFrameVector3D("toeRightLinearVelocity",
                                                                                                                   worldFrame,
                                                                                                                   controllerRegitry));

      YoFramePoint3D yoHeelPosition = new YoFramePoint3D("heelPosition", worldFrame, controllerRegitry);
      YoFrameVector3D yoHeelLinearVelocity = new YoFrameVector3D("heelLinearVelocity", worldFrame, controllerRegitry);
      SideDependentList<YoFramePoint3D> yoHeelCornerPositions = new SideDependentList<>(new YoFramePoint3D("heelLeftPosition", worldFrame, controllerRegitry),
                                                                                        new YoFramePoint3D("heelRightPosition", worldFrame, controllerRegitry));
      SideDependentList<YoFrameVector3D> yoHeelCornerLinearVelocities = new SideDependentList<>(new YoFrameVector3D("heelLeftLinearVelocity",
                                                                                                                    worldFrame,
                                                                                                                    controllerRegitry),
                                                                                                new YoFrameVector3D("heelRightLinearVelocity",
                                                                                                                    worldFrame,
                                                                                                                    controllerRegitry));

      return new MultiBodySystemStateReader()
      {
         private RigidBodyReadOnly foot;

         @Override
         public void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem)
         {
            foot = MultiBodySystemStateReader.findRigidBody("foot", multiBodySystem);
         }

         @Override
         public void read()
         {
            MovingReferenceFrame footFrame = foot.getBodyFixedFrame();
            yoFootCoMPose.set(footFrame.getTransformToRoot());
            TwistReadOnly footTwist = footFrame.getTwistOfFrame();
            yoFootCoMVelocity.getAngularPart().setMatchingFrame(footTwist.getAngularPart());
            yoFootCoMVelocity.getLinearPart().setMatchingFrame(footTwist.getLinearPart());

            FramePoint3D toe = new FramePoint3D(footFrame, 0.5 * footSize.getX(), 0, -0.5 * footSize.getZ());
            FramePoint3D heel = new FramePoint3D(footFrame, -0.5 * footSize.getX(), 0, -0.5 * footSize.getZ());
            FrameVector3D tempVector = new FrameVector3D();

            yoToePosition.setMatchingFrame(toe);
            yoHeelPosition.setMatchingFrame(heel);

            footTwist.getLinearVelocityAt(toe, tempVector);
            yoToeLinearVelocity.setMatchingFrame(tempVector);
            footTwist.getLinearVelocityAt(heel, tempVector);
            yoHeelLinearVelocity.setMatchingFrame(tempVector);

            for (RobotSide robotSide : RobotSide.values)
            {
               toe.setY(robotSide.negateIfRightSide(0.5 * footSize.getY()));
               heel.setY(robotSide.negateIfRightSide(0.5 * footSize.getY()));

               yoToeCornerPositions.get(robotSide).setMatchingFrame(toe);
               yoHeelCornerPositions.get(robotSide).setMatchingFrame(heel);

               footTwist.getLinearVelocityAt(toe, tempVector);
               yoToeCornerLinearVelocities.get(robotSide).setMatchingFrame(tempVector);
               footTwist.getLinearVelocityAt(heel, tempVector);
               yoHeelCornerLinearVelocities.get(robotSide).setMatchingFrame(tempVector);
            }
         }
      };
   }

   public MapBasedJointStateWriter createInitialStateWriter()
   {
      return new MapBasedJointStateWriter()
      {
         @Override
         public boolean write()
         {
            FloatingJointBasics rootJoint = getJoint(rootJointName);

            rootJoint.getJointPose().getPosition().setZ(comBallRadius + legLength + footSize.getZ() + 0.01);
            rootJoint.getJointTwist().getLinearPart().set(1.0, 0.5, 0.0);
            rootJoint.getJointWrench().getAngularPart().setZ(1.0);
            return true;
         }
      };
   }

   private RobotDescription createRobotDescription()
   {
      FloatingJointDescription rootJoint = new FloatingJointDescription(rootJointName);
      PinJointDescription anklePitchJoint = new PinJointDescription(anklePitchName, new Vector3D(0.0, 0.0, -comBallRadius - legLength), Axis3D.Y);
      PinJointDescription ankleRollJoint = new PinJointDescription(ankleRollName, new Vector3D(0.0, 0.0, -ankleJointSeparation), Axis3D.X);

      LinkDescription comBallLink = ExampleExperimentalSimulationTools.newSphereLink(comLinkName,
                                                                                     comBallRadius,
                                                                                     comBallMass,
                                                                                     0.8,
                                                                                     YoAppearance.CornflowerBlue(),
                                                                                     false,
                                                                                     null);
      comBallLink.getLinkGraphics().translate(0.0, 0.0, -comBallRadius - legLength);
      comBallLink.getLinkGraphics().addCylinder(legLength, 0.025, YoAppearance.LightCyan());
      LinkDescription ankleLink = new LinkDescription("ankleLink");
      ankleLink.setMass(0.25);
      ankleLink.setMomentOfInertia(0.001, 0.001, 0.001);
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addSphere(0.03, YoAppearance.Grey());
      ankleLink.setLinkGraphics(linkGraphics);
      LinkDescription footLink = ExampleExperimentalSimulationTools.newBoxLink(footLinkName, footSize, footMass, 0.8, footOffset, YoAppearance.DarkOrchid());

      rootJoint.setLink(comBallLink);
      anklePitchJoint.setLink(ankleLink);
      ankleRollJoint.setLink(footLink);

      RobotDescription robotDescription = new RobotDescription(robotName);
      robotDescription.addRootJoint(rootJoint);
      rootJoint.addJoint(anklePitchJoint);
      anklePitchJoint.addJoint(ankleRollJoint);

      for (RobotSide robotSide : RobotSide.values)
      {
         KinematicPointDescription toeLeft = new KinematicPointDescription("toe" + robotSide.getPascalCaseName(), footOffset);
         toeLeft.getOffsetFromJoint().addX(0.5 * footSize.getX());
         toeLeft.getOffsetFromJoint().addY(robotSide.negateIfRightSide(0.5 * footSize.getY()));
         toeLeft.getOffsetFromJoint().addZ(-0.5 * footSize.getZ());
         anklePitchJoint.addKinematicPoint(toeLeft);

         KinematicPointDescription heelLeft = new KinematicPointDescription("heel" + robotSide.getPascalCaseName(), footOffset);
         heelLeft.getOffsetFromJoint().addX(-0.5 * footSize.getX());
         heelLeft.getOffsetFromJoint().addY(robotSide.negateIfRightSide(0.5 * footSize.getY()));
         heelLeft.getOffsetFromJoint().addZ(-0.5 * footSize.getZ());
         anklePitchJoint.addKinematicPoint(heelLeft);
      }
      return robotDescription;
   }

   public static void main(String[] args) throws UnreasonableAccelerationException
   {
      new SimplifiedAnkleExperimentalSimulation();
   }
}
