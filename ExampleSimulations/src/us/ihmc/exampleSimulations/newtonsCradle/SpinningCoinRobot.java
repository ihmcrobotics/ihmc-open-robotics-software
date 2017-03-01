package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class SpinningCoinRobot
{
   private final double coinWidth = 0.00175; //quarter //0.1
   private final double coinRadius = 0.01213; //0.5; //
   private final double coinMass = 0.00567; //1.0; //
   private double spinningAngularVelocity = 10.0 * 2.0 * Math.PI;

   private final ArrayList<Robot> robots = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector linearMomentum = new YoFrameVector("linearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector angularMomentum = new YoFrameVector("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable translationalKineticEnergy = new DoubleYoVariable("translationalKineticEnergy", registry);
   private final DoubleYoVariable rotationalKineticEnergy = new DoubleYoVariable("rotationalKineticEnergy", registry);
   private final DoubleYoVariable potentialEnergy = new DoubleYoVariable("potentialEnergy", registry);
   private final DoubleYoVariable totalEnergy = new DoubleYoVariable("totalEnergy", registry);

   public SpinningCoinRobot()
   {
      final Robot coinRobot = new Robot("SpinningCoin");

      Vector3D offset = new Vector3D(0.0, 0.0, 0.0);
      FloatingJoint floatingJoint = new FloatingJoint("root", offset, coinRobot);

      Link link = createCylinderCoin(coinRobot);

      floatingJoint.setLink(link);
      coinRobot.addRootJoint(floatingJoint);

      double x = 0.1;
      double y = 0.1;
      double z = coinRadius + 0.04; //0.04;

      double yaw = 0.0;
      double pitch = 0.0;//0.0;
      double roll = 1.2; //Math.PI/2.0; //1.2;

      floatingJoint.setPosition(x, y, z);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

      //      floatingJoint.setVelocity(new Vector3d(1.0, 0.0, 0.0));
      floatingJoint.setAngularVelocityInBody(new Vector3D(0.0, spinningAngularVelocity, 0.0));

      robots.add(coinRobot);

      coinRobot.addFunctionToIntegrate(new FunctionToIntegrate()
      {
         private Vector3D tempVector = new Vector3D();

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
            coinRobot.computeAngularMomentum(tempVector);
            angularMomentum.set(tempVector);
            coinRobot.computeLinearMomentum(tempVector);
            linearMomentum.set(tempVector);

            potentialEnergy.set(coinRobot.computeGravitationalPotentialEnergy());
            translationalKineticEnergy.set(coinRobot.computeTranslationalKineticEnergy());
            rotationalKineticEnergy.set(coinRobot.computeRotationalKineticEnergy());

            totalEnergy.set(potentialEnergy.getDoubleValue());
            totalEnergy.add(translationalKineticEnergy);
            totalEnergy.add(rotationalKineticEnergy);

            return null;
         }
      });

      coinRobot.addYoVariableRegistry(registry);
   }

   private Link createCylinderCoin(Robot robot)
   {
      Link link = new Link("coin");
      link.setMassAndRadiiOfGyration(coinMass, coinRadius / 2.0, coinRadius / 2.0, coinWidth / 2.0);

      Matrix3D momentOfInertiaMatrix = new Matrix3D();
      link.getMomentOfInertia(momentOfInertiaMatrix);
      System.out.println("momentOfInertia = " + momentOfInertiaMatrix);

      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -coinWidth / 2.0);

      AppearanceDefinition color = YoAppearance.Purple();
      linkGraphics.addCylinder(coinWidth, coinRadius, color);
      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, coinWidth / 2.0);
      linkGraphics.addCube(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, YoAppearance.AliceBlue());
      linkGraphics.translate(0.0, 0.0, -coinWidth - coinWidth / 4.0);
      linkGraphics.addCube(coinRadius / 3.0, coinRadius / 3.0, coinWidth / 4.0, YoAppearance.Gold());
      //      link.addEllipsoidFromMassProperties(YoAppearance.DarkGreen());
      link.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMeshDescription = new CollisionMeshDescription();
      collisionMeshDescription.addCylinderReferencedAtCenter(coinRadius, coinWidth);
      collisionMeshDescription.setCollisionGroup(0xff);
      collisionMeshDescription.setCollisionMask(0xff);
      link.addCollisionMesh(collisionMeshDescription);

      return link;
   }

   public ArrayList<Robot> getRobots()
   {
      return robots;
   }

}
