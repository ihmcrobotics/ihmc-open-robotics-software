package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class StackOfBouncyBallsRobot extends Robot
{
   public StackOfBouncyBallsRobot()
   {
      super("StackOfBouncyBalls");

      double gravity = -9.81;
      this.setGravity(gravity);

      final YoFrameVector yoLinearMomentum = new YoFrameVector("linearMomentum", null, this.getRobotsYoVariableRegistry());
      final DoubleYoVariable potentialEnergy = new DoubleYoVariable("potentialEnergy", this.getRobotsYoVariableRegistry());
      final DoubleYoVariable kineticEnergy = new DoubleYoVariable("kineticEnergy", this.getRobotsYoVariableRegistry());
      final DoubleYoVariable totalEnergy = new DoubleYoVariable("totalEnergy", this.getRobotsYoVariableRegistry());

      int numberOfBalls = 4;
      double radiusScaleFactor = 0.6;
      double massScaleFactor = 0.2;

      double largestBallRadius = 0.25;
      double largestBallMass = 2000.0;

      double ballCenterHeight = 2.0 * largestBallRadius + 0.5;
      double ballRadius = largestBallRadius;
      double ballMass = largestBallMass;

      for (int i = 0; i < numberOfBalls; i++)
      {
         if (i != 0)
         {
            ballCenterHeight += ballRadius;
            ballRadius = ballRadius * radiusScaleFactor;
            ballCenterHeight += 1.01 * ballRadius + 0.05;
            ballMass = ballMass * massScaleFactor;
         }

         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("ball" + i, "ball" + i, offset, this);

         Link link = new Link("ball" + i);
         double ballRadiusOfGyration = ballRadius * 0.6;
         link.setMassAndRadiiOfGyration(ballMass, ballRadiusOfGyration, ballRadiusOfGyration, ballRadiusOfGyration);
         link.setComOffset(0.0, 0.0, 0.0);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addSphere(ballRadius, YoAppearance.Red());
         link.setLinkGraphics(linkGraphics);

         CollisionMeshDescription collisionMeshDescription = new CollisionMeshDescription();
         collisionMeshDescription.addSphere(ballRadius);
         link.setCollisionMesh(collisionMeshDescription);

         floatingJoint.setLink(link);
         this.addRootJoint(floatingJoint);

         floatingJoint.setPosition(0.0, 0.0, ballCenterHeight);

//         if (i==0)
//         {
//            floatingJoint.setVelocity(0.0, 0.0, 1.0);
//         }
//         else
//         {
//            floatingJoint.setVelocity(0.0, 0.0, -1.0);
//         }
      }

      FunctionToIntegrate functionToIntegrate = new FunctionToIntegrate()
      {
         @Override
         public double[] computeDerivativeVector()
         {
            Vector3d linearMomentum = new Vector3d();
            computeLinearMomentum(linearMomentum);
            kineticEnergy.set(computeTranslationalKineticEnergy());
            potentialEnergy.set(computeGravitationalPotentialEnergy());
            totalEnergy.set(kineticEnergy.getDoubleValue());
            totalEnergy.add(potentialEnergy);
            yoLinearMomentum.set(linearMomentum);

            return null;
         }

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
      };

      this.addFunctionToIntegrate(functionToIntegrate);
   }

}
