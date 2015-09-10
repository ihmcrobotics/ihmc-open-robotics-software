package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class RowOfDominosRobot extends Robot
{
   private final ScsCollisionDetector collisionDetector;

   public RowOfDominosRobot()
   {
      super("RowOfDominosRobot");
      
      final YoFrameVector yoLinearMomentum = new YoFrameVector("linearMomentum", null, this.getRobotsYoVariableRegistry()); 
      final DoubleYoVariable potentialEnergy = new DoubleYoVariable("potentialEnergy", this.getRobotsYoVariableRegistry());
      final DoubleYoVariable kineticEnergy = new DoubleYoVariable("kineticEnergy", this.getRobotsYoVariableRegistry());
      final DoubleYoVariable totalEnergy = new DoubleYoVariable("totalEnergy", this.getRobotsYoVariableRegistry());
      
      int numberOfDominos = 10;

      collisionDetector = new GdxCollisionDetector(this.getRobotsYoVariableRegistry(), 10000);
      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      double dominoWidth = 0.024;
      double dominoDepth = 0.0075;
      double dominoHeight = 0.048;
      
      double dominoMass = 0.2;
      RigidBodyTransform nextDominoTransform = new RigidBodyTransform();
      nextDominoTransform.setTranslation(new Vector3d(0.0, 0.0, dominoHeight/2.0 + 0.006));
      
      Vector3d dominoTranslation = new Vector3d(dominoHeight * 0.6, 0.0, 0.0);
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      RigidBodyTransform tempTransform2 = new RigidBodyTransform();

      for (int i = 0; i < numberOfDominos; i++)
      {
         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("domino" + i, "domino" + i, offset, this);

         Link link = new Link("domino" + i);
         link.setMassAndRadiiOfGyration(dominoMass, dominoDepth/2.0, dominoWidth/2.0, dominoHeight/2.0);
         link.setComOffset(0.0, 0.0, 0.0);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.translate(0.0, 0.0, -dominoHeight/2.0);
         linkGraphics.addCube(dominoDepth, dominoWidth, dominoHeight, YoAppearance.Red());
         link.setLinkGraphics(linkGraphics);

         CollisionShapeDescription shapeDesc = collisionShapeFactory.createBox(dominoDepth/2.0, dominoWidth/2.0, dominoHeight/2.0);
         RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
         shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
         collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
         link.enableCollisions(100.0, this.getRobotsYoVariableRegistry());

         floatingJoint.setLink(link);
         this.addRootJoint(floatingJoint);

         
         floatingJoint.setRotationAndTranslation(nextDominoTransform);
         
         tempTransform.setIdentity();
         tempTransform.rotZ(0.15);
         tempTransform.setTranslation(dominoTranslation);
         
         tempTransform2.setIdentity();
         tempTransform2.multiply(tempTransform, nextDominoTransform);
         nextDominoTransform.set(tempTransform2);
         
         if (i==0)
         {
            floatingJoint.setAngularVelocityInBody(new Vector3d(0.0, 10.0, 0.0));
         }         
      }


      NullJoint baseJoint = new NullJoint("base", new Vector3d(), this);

//    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(1000000000.0, 100.0, 100.0, 100.0);
      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -0.01);
      baseLinkGraphics.addCube(100.0, 100.0, 0.01, YoAppearance.Green());
      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.enableCollisions(100.0, this.getRobotsYoVariableRegistry());

      CollisionShapeDescription shapeDesc = collisionShapeFactory.createBox(100.0, 100.0, 0.01/2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(-0.005, 0.0, 0.0));
      collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);

//    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseJoint.setLink(baseLink);
      this.addRootJoint(baseJoint);

      this.addStaticLink(baseLink);

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

   public ScsCollisionDetector getCollisionDetector()
   {
      return collisionDetector;
   }

}
