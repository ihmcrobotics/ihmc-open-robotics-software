package us.ihmc.commonWalkingControlModules.controlModules;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class ChestOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector feedForwardAngularAcceleration;
   private final RigidBody chest;

   public ChestOrientationControlModule(RigidBody pelvis, RigidBody chest, TwistCalculator twistCalculator,
                                        double controlDT, YoVariableRegistry parentRegistry)
   {
      super("chest", new RigidBody[] {}, chest, new GeometricJacobian[]{}, twistCalculator, controlDT, parentRegistry);

      this.chest = chest;
      ReferenceFrame baseFrame = pelvis.getBodyFixedFrame();
      this.desiredOrientation = new YoFrameQuaternion("desiredChestOrientation", baseFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector("desiredChestAngularVelocity", baseFrame, registry);
      this.feedForwardAngularAcceleration = new YoFrameVector("desiredChestAngularAcceleration", baseFrame, registry);
   }

   public RigidBody getChest()
   {
      return chest;
   }

   protected FrameOrientation getDesiredFrameOrientationCopy()
   {
      FrameOrientation ret = new FrameOrientation(desiredOrientation.getReferenceFrame());
      desiredOrientation.get(ret);
      return ret;
   }

   protected FrameVector getDesiredAngularVelocityCopy()
   {
      return desiredAngularVelocity.getFrameVectorCopy();
   }

   protected FrameVector getDesiredAngularAccelerationFeedForwardCopy()
   {
      return feedForwardAngularAcceleration.getFrameVectorCopy();
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(this.desiredOrientation.getReferenceFrame());
      this.desiredOrientation.set(desiredOrientation);

      desiredAngularVelocity.changeFrame(this.desiredAngularVelocity.getReferenceFrame());
      this.desiredAngularVelocity.set(desiredAngularVelocity);

      feedForwardAngularAcceleration.changeFrame(this.feedForwardAngularAcceleration.getReferenceFrame());
      this.feedForwardAngularAcceleration.set(feedForwardAngularAcceleration);
   }
}
