package us.ihmc.commonWalkingControlModules.visualizer;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class MomentumVisualizer implements Updatable
{
   private final CenterOfMassCalculator comCalculator;
   private final MomentumCalculator momentumCalculator;
   private final YoFramePoint centerOfMass;
   private final YoFrameVector linearMomentum;

   private final Momentum momentum = new Momentum(ReferenceFrame.getWorldFrame());
   private final FrameVector3D frameVector = new FrameVector3D();

   public MomentumVisualizer(String name, OneDoFJoint rootJoint, YoVariableRegistry registry,
         YoGraphicsListRegistry graphicsRegistry)
   {
      this(name, registry, graphicsRegistry, ScrewTools.computeRigidBodiesAfterThisJoint(rootJoint));
   }

   public MomentumVisualizer(String name, YoVariableRegistry registry, YoGraphicsListRegistry graphicsRegistry,
         RigidBody... rigidBodies)
   {
      comCalculator = new CenterOfMassCalculator(rigidBodies, ReferenceFrame.getWorldFrame());
      momentumCalculator = new MomentumCalculator(rigidBodies);
      centerOfMass = new YoFramePoint(name + "CoM", ReferenceFrame.getWorldFrame(), registry);
      linearMomentum = new YoFrameVector(name + "Momentum", ReferenceFrame.getWorldFrame(), registry);

      YoGraphicPosition yoCoMGraphics = new YoGraphicPosition(name + "CoM", centerOfMass, 0.05, YoAppearance.Brown());
      YoGraphicVector yoMomentumGraphics = new YoGraphicVector(name + "Momentum", centerOfMass, linearMomentum, 0.05, YoAppearance.Brown());
      graphicsRegistry.registerYoGraphic(name, yoCoMGraphics);
      graphicsRegistry.registerYoGraphic(name, yoMomentumGraphics);
   }

   @Override
   public void update(double time)
   {
      comCalculator.compute();
      centerOfMass.set(comCalculator.getCenterOfMass());

      momentumCalculator.computeAndPack(momentum);
      momentum.getLinearPart(frameVector);
      linearMomentum.set(frameVector);
   }
}
