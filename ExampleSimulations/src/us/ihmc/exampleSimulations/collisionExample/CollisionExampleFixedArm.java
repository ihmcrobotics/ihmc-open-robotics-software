package us.ihmc.exampleSimulations.collisionExample;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;

public class CollisionExampleFixedArm
{

   public static class DoublePendulumRobot extends Robot
   {
      private final DoubleYoVariable potentialEnergy = new DoubleYoVariable("potentialEnergy", getRobotsYoVariableRegistry());
      private final DoubleYoVariable linearKineticEnergy = new DoubleYoVariable("linearKineticEnergy", getRobotsYoVariableRegistry());
      private final DoubleYoVariable rotationalKineticEnergy = new DoubleYoVariable("rotationalKineticEnergy", getRobotsYoVariableRegistry());
      private final DoubleYoVariable totalEnergy = new DoubleYoVariable("totalEnergy", getRobotsYoVariableRegistry());

      //      ScsCollisionDetector collisionDetector = new JBulletCollisionDetector(getRobotsYoVariableRegistry(), 10000);
      private final ScsCollisionDetector collisionDetector = new GdxCollisionDetector(10000.0);
      private final CollisionHandler collisionHandler;

      /*
       * L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1
       * and R2 are the radii of the links, Iyy1 and Iyy2 are the moments of
       * inertia of the links. The moments of inertia are defined about the COM
       * for each link.
       */
      private static final double L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, M3 = 1, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;

      private final PinJoint pin1, pin2;

      public DoublePendulumRobot()
      {
         super("DoublePendulum"); // create and instance of Robot
         // Create joints and assign links. Pin joints have a single axis of rotation.
         pin1 = new PinJoint("joint1", new Vector3d(0.0, 0.0, 0), this, Axis.Y);
         // pin1.setInitialState(0.0, 0.0);
         Link link1 = link1();
         pin1.setLink(link1); // associate link1 with the joint pin1
         this.addRootJoint(pin1);

         pin1.setInitialState(0, 0.001);

         /*
          * The second joint is initiated with the offset vector (0.0,0.0,L1)
          * since it should be placed a distance of L1 in the Z direction from
          * the previous joint.
          */
         pin2 = new PinJoint("joint2", new Vector3d(0.0, 0.0, L1), this, Axis.Y);
         Link link2 = link2();

         Joint pin3 = new PinJoint("joint3", new Vector3d(0.0, 0.0, L2), this, Axis.Y);
         Link linkBlock = endBlock();

         pin3.setLink(linkBlock);
         pin2.addJoint(pin3);
         pin2.setLink(link2);
         pin1.addJoint(pin2);

         // add the ground
         addRootJoint(groundPlane());


         FunctionToIntegrate functionToIntegrate = new FunctionToIntegrate()
         {
            @Override
            public int getVectorSize()
            {
               return 0;
            }

            @Override
            public DoubleYoVariable[] getOutputVariables()
            {
               return null;
            }

            @Override
            public double[] computeDerivativeVector()
            {
               potentialEnergy.set(computeGravitationalPotentialEnergy(pin1));
               linearKineticEnergy.set(computeTranslationalKineticEnergy(pin1));
               rotationalKineticEnergy.set(computeRotationalKineticEnergy(pin1));

               totalEnergy.set(potentialEnergy.getDoubleValue());
               totalEnergy.add(linearKineticEnergy);
               totalEnergy.add(rotationalKineticEnergy);

               return null;
            }
         };

         this.addFunctionToIntegrate(functionToIntegrate);

         //         collisionHandler = new SpringCollisionHandler(1, 1000, 3, this.getRobotsYoVariableRegistry());

         double epsilon = 0.9;
         double mu = 0.5;
         collisionHandler = new DefaultCollisionHandler(epsilon, mu);
         collisionDetector.initialize();
      }

      /**
       * Create the first link for the DoublePendulumRobot.
       */
      private Link link1()
      {
         Link ret = new Link("link1");
         ret.setMass(M1);
         ret.setComOffset(0.0, 0.0, L1 / 2.0);
         ret.setMomentOfInertia(0.0, Iyy1, 0.0);
         // create a LinkGraphics object to manipulate the visual representation of the link
         Graphics3DObject linkGraphics = new Graphics3DObject();
         //         linkGraphics.translate(0.0, 0.0, L1/2);
         linkGraphics.addCylinder(L1, R1, YoAppearance.Red());

         // associate the linkGraphics object with the link object
         ret.setLinkGraphics(linkGraphics);
         return ret;
      }

      /**
       * Create the second link for the DoublePendulumRobot.
       */
      private Link link2()
      {
         Link ret = new Link("link2");
         ret.setMass(M2);
         ret.setComOffset(0.0, 0.0, L2 / 2.0);
         ret.setMomentOfInertia(0.0, Iyy2, 0.0);
         Graphics3DObject linkGraphics = new Graphics3DObject();
         //         linkGraphics.translate(0.0, 0.0, 0.05);
         linkGraphics.addCylinder(L2, R2, YoAppearance.Green());
         ret.setLinkGraphics(linkGraphics);
         return ret;
      }

      private Link endBlock()
      {
         double size = 0.2;

         Link ret = new Link("EndBlock");
         ret.setMass(M3);
         ret.setMomentOfInertia(0.1 * M3, 0.1 * M3, 0.1 * M3);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.translate(0.0, 0.0, 0.0);
         linkGraphics.addCube(size, size, size, YoAppearance.Blue());
         ret.setLinkGraphics(linkGraphics);

         CollisionShapeFactory factory = collisionDetector.getShapeFactory();
         factory.setMargin(0.02);
         CollisionShapeDescription shapeDesc = factory.createBox(size / 2.0, size / 2.0, size / 2.0);
         factory.addShape(ret, null, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);

         ret.enableCollisions(this.getRobotsYoVariableRegistry());
         return ret;
      }

      private FloatingJoint groundPlane()
      {
         FloatingJoint groundJoint = new FloatingJoint("ground", "ground", new Vector3d(), this);

         Link ground = new Link("ground");

         double width = 40;
         double height = 0.2;
         double mass = 10000;
         ground.setMass(mass);
         ground.setMomentOfInertia(0.1 * mass, 0.1 * mass, 0.1 * mass);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.translate(0.0, 0.0, 0.0);
         linkGraphics.addCube(width, width, height, YoAppearance.Beige());
         ground.setLinkGraphics(linkGraphics);
         ground.enableCollisions(this.getRobotsYoVariableRegistry());

         groundJoint.setLink(ground);
         groundJoint.setPositionAndVelocity(0.0, 0.0, -height, 0.0, 0.0, 0.0);
         groundJoint.setDynamic(false);

         CollisionShapeFactory factory = collisionDetector.getShapeFactory();
         factory.setMargin(0.02);
         CollisionShapeDescription shapeDesc = factory.createBox(width / 2, width / 2, height / 2);
         factory.addShape(ground, null, shapeDesc, true, 0xFFFFFFFF, 0xFFFFFFFF);

         return groundJoint;
      }

      public ScsCollisionDetector getCollisionDetector()
      {
         return collisionDetector;
      }

      public CollisionHandler getCollisionHandler()
      {
         return collisionHandler;
      }
   }

   public static void main(String[] args)
   {
      DoublePendulumRobot doublePendulum = new DoublePendulumRobot();
      SimulationConstructionSet sim = new SimulationConstructionSet(doublePendulum);
      sim.setDT(0.0001, 10);
//      sim.setFastSimulate(true);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -40.0, 0);

      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(0.1, 0.1, sim, 100);

      CollisionHandler collisionHandler = doublePendulum.getCollisionHandler();
      collisionHandler.addListener(visualize);

      sim.initPhysics(new ScsPhysics(null, doublePendulum.getCollisionDetector(), collisionHandler, visualize));

      Thread myThread = new Thread(sim);
      myThread.start();
   }
}
