package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;

public class NewtonsCradleExperimentalSimulation
{
   private static final String NEWTONS_CRADLE = "NewtonsCradle";
   private final ContactParameters contactParameters = new ContactParameters(0.0, 1.0, 0.0, 1.0);
   private final int numberOfBalls = 6;
   private final double ballRadius = 0.05;

   private final double stringLength = 0.6;
   private final double stringRadius = 0.002;
   private final double ballMass = 0.2;

   public NewtonsCradleExperimentalSimulation()
   {
      RobotDescription robotDescription = new RobotDescription(NEWTONS_CRADLE);
      double ballRadiusOfGyration = ballRadius * 0.6;
      double pinJointHeight = 1.1 * stringLength;
      double pinJointSeparation = 2.001 * ballRadius;

      for (int i = 0; i < numberOfBalls; i++)
      {
         Vector3D offset = new Vector3D(i * pinJointSeparation, 1.0, pinJointHeight);
         PinJointDescription pinJoint = new PinJointDescription("pin" + i, offset, Axis3D.Y);

         LinkDescription link = new LinkDescription(getBallBodyName(i));
         link.setMassAndRadiiOfGyration(ballMass, ballRadiusOfGyration, ballRadiusOfGyration, ballRadiusOfGyration);
         link.setCenterOfMassOffset(0.0, 0.0, -stringLength);

         LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
         linkGraphics.translate(0.0, 0.0, -stringLength);
         linkGraphics.addCylinder(stringLength, stringRadius, YoAppearance.Yellow());
         AppearanceDefinition aliceBlue = YoAppearance.Red();
         aliceBlue.setTransparency(0.4);
         linkGraphics.addSphere(ballRadius, aliceBlue);
         link.setLinkGraphics(linkGraphics);

         pinJoint.setLink(link);
         robotDescription.addRootJoint(pinJoint);
      }

      CollidableHelper helper = new CollidableHelper();

      RobotCollisionModel robotCollisionModel = new RobotCollisionModel()
      {
         @Override
         public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
         {
            List<Collidable> collidables = new ArrayList<>();

            for (int i = 0; i < numberOfBalls; i++)
            {
               long collisionMask = helper.getCollisionMask(getBallBodyName(i));
               long collisionGroup = helper.createCollisionGroup(getOtherBallBodyNames(i));
               RigidBodyBasics ballBody = RobotCollisionModel.findRigidBody(getBallBodyName(i), multiBodySystem);
               FrameSphere3D ballShape = new FrameSphere3D(ballBody.getBodyFixedFrame(), ballRadius);
               collidables.add(new Collidable(ballBody, collisionMask, collisionGroup, ballShape));
            }

            return collidables;
         }
      };

      RobotFromDescription robot = new RobotFromDescription(robotDescription);
      RigidBodyBasics rootBody = toInverseDynamicsRobot(robotDescription);

      MultiBodySystemStateWriter robotInitialStateWriter = new MultiBodySystemStateWriter()
      {
         private List<OneDoFJointBasics> joints;

         @Override
         public void write()
         {
            joints.get(0).setQ(0.3);
            joints.get(1).setQ(0.3);
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            joints = multiBodySystem.getAllJoints().stream().map(OneDoFJointBasics.class::cast).collect(Collectors.toList());
         }
      };

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(new Robot[] {robot}, 1 << 16);
      experimentalSimulation.setDT(simDT, 1);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(robot.getName(), rootBody, robotCollisionModel, robotInitialStateWriter);

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      experimentalSimulation.simulate();
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.startOnAThread();
      scs.simulate(5.0);//2.1125);
   }

   private String getBallBodyName(int i)
   {
      return "ball" + i;
   }

   private String[] getOtherBallBodyNames(int i)
   {
      String[] otherNames = new String[numberOfBalls - 1];
      int index = 0;
      for (int j = 0; j < numberOfBalls; j++)
      {
         if (j != i)
            otherNames[index++] = getBallBodyName(j);
      }
      return otherNames;
   }

   private RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      return rootBody;
   }

   private void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      Joint joint;

      if (jointDescription instanceof PinJointDescription)
      {
         String name = jointDescription.getName();
         Vector3D jointOffset = new Vector3D();
         jointDescription.getOffsetFromParentJoint(jointOffset);
         Vector3D jointAxis = new Vector3D();
         ((PinJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new RevoluteJoint(name, parentBody, jointOffset, jointAxis);
      }
      else
      {
         throw new IllegalStateException("Joint type not handled.");
      }

      LinkDescription linkDescription = jointDescription.getLink();

      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      RigidBody successor = new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);
      joint.setSuccessor(successor);

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }

   public static void main(String[] args)
   {
      new NewtonsCradleExperimentalSimulation();
   }
}
