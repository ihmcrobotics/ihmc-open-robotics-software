package us.ihmc.commonWalkingControlModules.visualizer;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class CenterOfMassVisualizer implements Updatable
{
   CenterOfMassCalculator comCalculator;
   MomentumCalculator momentumCalculator;
   YoFramePoint yoPoint;
   YoFrameVector yoVector;
   YoGraphicVector yoMomentumGraphics;
   YoGraphicPosition yoCoMGraphics;

   public CenterOfMassVisualizer(String name, OneDoFJoint rootJoint, TwistCalculator twistCalculator, YoVariableRegistry registry, YoGraphicsListRegistry graphicsRegistry)
   {
      this(name, twistCalculator, registry, graphicsRegistry, ScrewTools.computeRigidBodiesAfterThisJoint(rootJoint));
   }

   public CenterOfMassVisualizer(String name, TwistCalculator twistCalculator, YoVariableRegistry registry, YoGraphicsListRegistry graphicsRegistry, RigidBody... rigidBodies)
   {
      comCalculator = new CenterOfMassCalculator(rigidBodies, ReferenceFrame.getWorldFrame());
      momentumCalculator = new MomentumCalculator(twistCalculator, rigidBodies);
      yoPoint = new YoFramePoint(name+"CoM", ReferenceFrame.getWorldFrame(), registry);
      yoVector = new YoFrameVector(name+"Momentum", ReferenceFrame.getWorldFrame(), registry);

      yoCoMGraphics = new YoGraphicPosition(name+"CoM", yoPoint, 0.05, YoAppearance.Brown());
      yoMomentumGraphics = new YoGraphicVector(name+"Momentum", yoPoint, yoVector, 0.05, YoAppearance.Brown());
      graphicsRegistry.registerYoGraphic(name, yoCoMGraphics);
      graphicsRegistry.registerYoGraphic(name, yoMomentumGraphics);

   }

   Momentum momentum = new Momentum(ReferenceFrame.getWorldFrame());
   FrameVector frameVector = new FrameVector();
   @Override
   public void update(double time)
   {
      comCalculator.compute();
      yoPoint.set(comCalculator.getCenterOfMass());

      momentumCalculator.computeAndPack(momentum);
      momentum.packLinearPart(frameVector);
      yoVector.set(frameVector);

   }

}
