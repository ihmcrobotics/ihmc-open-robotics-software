package us.ihmc.commonWalkingControlModules.controlModules.foot.wobble;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootRotationHelper
{
   private final RobotSide side;
   private final FootRotationInformation rotationInformation;
   private final FixedFramePoint3DBasics desiredCop;

   public FootRotationHelper(RobotSide side, ReferenceFrame soleFrame, FootRotationInformation rotationInformation, YoVariableRegistry parentRegistry,
                             YoGraphicsListRegistry graphicsRegistry)
   {
      this.side = side;
      this.rotationInformation = rotationInformation;

      desiredCop = new FramePoint3D(soleFrame);

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);
   }

   public void compute(FrameLine2DReadOnly lineOfRotation)
   {
      desiredCop.setToZero();
      rotationInformation.setRotating(side, desiredCop);
   }

   public void reset()
   {
      rotationInformation.reset(side);
   }
}
