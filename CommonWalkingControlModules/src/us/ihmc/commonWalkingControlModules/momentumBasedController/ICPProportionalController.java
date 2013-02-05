package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2d tempControl = new FrameVector2d(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final DoubleYoVariable alphaICPError = new DoubleYoVariable("alphaICPError", registry);
   private final AlphaFilteredYoFrameVector2d filteredICPError = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d("filteredICPError", "",
                                                                    registry, alphaICPError, icpError);
   private final double controlDT;
   private final DoubleYoVariable captureKp = new DoubleYoVariable("captureKp", registry);

   public ICPProportionalController(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      filteredICPError.reset();
   }

   public FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity,
           double omega0)
   {
      desiredCapturePointVelocity.changeFrame(desiredCapturePoint.getReferenceFrame());
      FramePoint2d desiredCMP = new FramePoint2d(capturePoint);

      // feed forward part
      tempControl.setAndChangeFrame(desiredCapturePointVelocity);
      tempControl.scale(1.0 / omega0);
      desiredCMP.sub(tempControl);

      // feedback part
      tempControl.setAndChangeFrame(capturePoint);
      tempControl.sub(desiredCapturePoint);
      icpError.set(tempControl);
      filteredICPError.update();

      filteredICPError.getFrameVector2d(tempControl);
      tempControl.scale(captureKp.getDoubleValue());
      desiredCMP.add(tempControl);

      return desiredCMP;
   }

   public void setGains(double captureKp, double filterBreakFrequencyHertz)
   {
      this.captureKp.set(captureKp);
      this.alphaICPError.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequencyHertz, controlDT));
   }
}
