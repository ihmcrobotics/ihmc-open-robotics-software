package us.ihmc.commonWalkingControlModules.controlModules;

import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class ChestOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameOrientation desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector feedForwardAngularAcceleration;

   public ChestOrientationControlModule(RigidBody pelvis, RigidBody chest, GeometricJacobian jacobian, TwistCalculator twistCalculator)
   {
      super("chest", pelvis, chest, jacobian, twistCalculator);
      ReferenceFrame baseFrame = pelvis.getBodyFixedFrame();
      this.desiredOrientation = new YoFrameOrientation("desiredChestOrientation", baseFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector("desiredChestAngularVelocity", baseFrame, registry);
      this.feedForwardAngularAcceleration = new YoFrameVector("desiredChestAngularAcceleration", baseFrame, registry);
   }

   protected FrameOrientation getDesiredFrameOrientation()
   {
      return desiredOrientation.getFrameOrientationCopy();
   }

   protected FrameVector getDesiredAngularVelocity()
   {
      return desiredAngularVelocity.getFrameVectorCopy();
   }

   protected FrameVector getDesiredAngularAccelerationFeedForward()
   {
      return feedForwardAngularAcceleration.getFrameVectorCopy();
   }
   
   // TODO: add setters for Yo stuff once we need them
}
