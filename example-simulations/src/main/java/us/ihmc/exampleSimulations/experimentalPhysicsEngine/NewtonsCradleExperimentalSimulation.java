package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;

public class NewtonsCradleExperimentalSimulation
{
   private static final String NEWTONS_CRADLE = "NewtonsCradle";
   private final ContactParameters contactParameters = new ContactParameters();
   private final int numberOfBalls = 6;
   private final double ballRadius = 0.05;

   private final double stringLength = 0.6;
   private final double stringRadius = 0.002;
   private final double ballMass = 0.2;

   public NewtonsCradleExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfRestitution(1.0);
      contactParameters.setRestitutionThreshold(0.0);

      RobotDescription robotDescription = new RobotDescription(NEWTONS_CRADLE);
      double ballRadiusOfGyration = ballRadius * 0.6;
      double pinJointHeight = 1.1 * stringLength;
      double pinJointSeparation = 2.0001 * ballRadius; // FIXME Note the 1.0e-4 epsilon here, this is to prevent things from blowing up. Need to figure out how to fix this.

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

      MultiBodySystemStateWriter robotInitialStateWriter = new MultiBodySystemStateWriter()
      {
         private List<OneDoFJointBasics> joints;

         @Override
         public boolean write()
         {
            joints.get(0).setQ(0.3);
            joints.get(1).setQ(0.3);
            return true;
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            joints = multiBodySystem.getAllJoints().stream().map(OneDoFJointBasics.class::cast).collect(Collectors.toList());
         }
      };

      double simDT = 0.0001;
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(1 << 16);
      experimentalSimulation.setDT(simDT, 1);
      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -9.81));
      experimentalSimulation.getPhysicsEngine().setGlobalContactParameters(contactParameters);
      experimentalSimulation.addRobot(robotDescription, robotCollisionModel, robotInitialStateWriter);
      experimentalSimulation.addSimulationEnergyStatistics();

      SimulationConstructionSet scs = new SimulationConstructionSet(experimentalSimulation,
                                                                    SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true),
                                                                    parameters);
      experimentalSimulation.simulate();
      scs.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
      scs.setDT(simDT, 1);
      scs.setFastSimulate(true);
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

   public static void main(String[] args)
   {
      new NewtonsCradleExperimentalSimulation();
   }
}
