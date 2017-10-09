package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.ExternalTorque;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.physics.CollisionShape;

public class DefaultCollisionVisualizer implements CollisionHandlerListener
{
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private List<YoGraphicVector> activeRedYoGraphicVectors = new ArrayList<YoGraphicVector>();
   private List<YoGraphicVector> activeBlueYoGraphicVectors = new ArrayList<YoGraphicVector>();

   private BagOfBalls collisionPositionsVizOne, collisionPositionsVizTwo;
   private int total;

   private YoVariableRegistry registry;

   private double impulseScale = 0.1;
   private double forceScale = 0.005;
   private double collisionBallRadius = 0.01;

   public DefaultCollisionVisualizer(double forceScale, double impulseScale, double collisionBallRadius, SimulationConstructionSet scs, int numberOfVectorsToCreate)
   {
      this.forceScale = forceScale;
      this.impulseScale = impulseScale;
      this.collisionBallRadius = collisionBallRadius;
      initialize(scs, numberOfVectorsToCreate);
   }

   private void initialize(SimulationConstructionSet scs, int numberOfVectorsToCreate)
   {
      this.scs = scs;

      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      registry = new YoVariableRegistry(getClass().getSimpleName());
      rootRegistry.addChild(registry);

      collisionPositionsVizOne = new BagOfBalls(50, collisionBallRadius, "CollisionOne", YoAppearance.Red(), registry, yoGraphicsListRegistry);
      collisionPositionsVizTwo = new BagOfBalls(50, collisionBallRadius, "CollisionTwo", YoAppearance.Blue(), registry, yoGraphicsListRegistry);

      for (int i = 0; i < numberOfVectorsToCreate; i++)
      {
         addImpulseYoGraphicVector(i * 2, true, impulseScale);
         addImpulseYoGraphicVector(i * 2 + 1, false, impulseScale);
      }

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
   }

   public void callBeforeCollisionDetection()
   {
//      collisionPositionsViz.reset();
      
      //      System.out.println("CallBeforeCollision");
      //      for (int i = 0; i < total; i++)
      //      {
      //         setToInfinity(activeRedYoGraphicVectors.get(i));
      //         setToInfinity(activeBlueYoGraphicVectors.get(i));
      //      }

      total = 0;
   }

   @Override
   public void collision(CollisionShape shapeA, CollisionShape shapeB, ExternalForcePoint forceA, ExternalForcePoint forceB, ExternalTorque torqueA,
         ExternalTorque torqueB)
   {
      if (total >= activeRedYoGraphicVectors.size())
         return;

      YoGraphicVector yoGraphicVectorA = activeRedYoGraphicVectors.get(total);
      YoGraphicVector yoGraphicVectorB = activeBlueYoGraphicVectors.get(total);

      //      System.out.println(forceA);

      //      yoGraphicVectorA.set(forceA.getYoPosition(), forceA.getYoForce());
      //      yoGraphicVectorB.set(forceB.getYoPosition(), forceB.getYoForce());

      //      System.out.println("Visualizing Collision. forceA = " + forceA);
      //      System.out.println("Visualizing Collision. forceB = " + forceB);
      //
      //      System.out.println("Visualizing Collision. forceA.getYoPosition() = " + forceA.getYoPosition());
      //      System.out.println("Visualizing Collision. forceA.getYoImpulse() = " + forceA.getYoImpulse());
      //      System.out.println("Visualizing Collision. forceB.getYoPosition() = " + forceB.getYoPosition());
      //      System.out.println("Visualizing Collision. forceB.getYoImpulse() = " + forceB.getYoImpulse());

      collisionPositionsVizOne.setBallLoop(forceA.getYoPosition().getFrameTuple());
      collisionPositionsVizTwo.setBallLoop(forceB.getYoPosition().getFrameTuple());

      yoGraphicVectorA.set(forceA.getYoPosition(), forceA.getYoImpulse());
      yoGraphicVectorB.set(forceB.getYoPosition(), forceB.getYoImpulse());
      total++;
   }

   public void addImpulseYoGraphicVector(int num, boolean isRed, double scale)
   {
      List<YoGraphicVector> active = isRed ? activeRedYoGraphicVectors : activeBlueYoGraphicVectors;

      YoDouble yo0 = new YoDouble("locX_" + num, registry);
      YoDouble yo1 = new YoDouble("locY_" + num, registry);
      YoDouble yo2 = new YoDouble("locZ_" + num, registry);
      YoDouble yo3 = new YoDouble("vecX_" + num, registry);
      YoDouble yo4 = new YoDouble("vecY_" + num, registry);
      YoDouble yo5 = new YoDouble("vecZ_" + num, registry);

      String name = "ContactVisualized" + num;
      AppearanceDefinition appearance = isRed ? YoAppearance.Red() : YoAppearance.Blue();
      YoGraphicVector yoGraphicVector = new YoGraphicVector(name, yo0, yo1, yo2, yo3, yo4, yo5, scale, appearance);
      yoGraphicVector.setLineRadiusWhenOneMeterLong(0.005);
      yoGraphicVector.setMinAndMaxRadiusScaleFactors(0.01, 1.5);

      yoGraphicsListRegistry.registerYoGraphic("CollisionVectors", yoGraphicVector);

      //      scs.addYoGraphic(yoGraphicVector, true);
      active.add(yoGraphicVector);
   }

   public void setToInfinity(YoGraphicVector yoGraphicVector)
   {
      double largeNumber = 1000000;
      yoGraphicVector.set(largeNumber, largeNumber, largeNumber, 0, 0, 0);
   }

   public void setForceScale(double forceScale)
   {
      this.forceScale = forceScale;
   }
}
