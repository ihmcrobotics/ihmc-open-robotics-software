package us.ihmc.commonWalkingControlModules.visualizer;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MomentumVisualizer implements Updatable
{
   private final CenterOfMassCalculator comCalculator;
   private final MomentumCalculator momentumCalculator;
   private final YoFramePoint3D centerOfMass;
   private final YoFrameVector3D linearMomentum;

   private final Momentum momentum = new Momentum(ReferenceFrame.getWorldFrame());
   private final FrameVector3D frameVector = new FrameVector3D();

   public MomentumVisualizer(String name, OneDoFJointBasics rootJoint, YoRegistry registry,
         YoGraphicsListRegistry graphicsRegistry)
   {
      this(name, registry, graphicsRegistry, rootJoint.getSuccessor());
   }

   public MomentumVisualizer(String name, YoRegistry registry, YoGraphicsListRegistry graphicsRegistry,
         RigidBodyBasics rootBody)
   {
      comCalculator = new CenterOfMassCalculator(rootBody, ReferenceFrame.getWorldFrame());
      momentumCalculator = new MomentumCalculator(rootBody);
      centerOfMass = new YoFramePoint3D(name + "CoM", ReferenceFrame.getWorldFrame(), registry);
      linearMomentum = new YoFrameVector3D(name + "Momentum", ReferenceFrame.getWorldFrame(), registry);

      YoGraphicPosition yoCoMGraphics = new YoGraphicPosition(name + "CoM", centerOfMass, 0.05, YoAppearance.Brown());
      YoGraphicVector yoMomentumGraphics = new YoGraphicVector(name + "Momentum", centerOfMass, linearMomentum, 0.05, YoAppearance.Brown());
      graphicsRegistry.registerYoGraphic(name, yoCoMGraphics);
      graphicsRegistry.registerYoGraphic(name, yoMomentumGraphics);
   }

   @Override
   public void update(double time)
   {
      comCalculator.reset();
      centerOfMass.set(comCalculator.getCenterOfMass());

      momentumCalculator.computeAndPack(momentum);
      frameVector.set(momentum.getLinearPart());
      linearMomentum.set(frameVector);
   }
}
