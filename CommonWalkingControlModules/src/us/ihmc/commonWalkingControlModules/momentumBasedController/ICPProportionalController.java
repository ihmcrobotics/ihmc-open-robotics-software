package us.ihmc.commonWalkingControlModules.momentumBasedController;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.MathTools;
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
   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);
   private final DoubleYoVariable alphaFeedBack = new DoubleYoVariable("alphaFeedBack", registry);
   private final AlphaFilteredYoFrameVector2d filteredFeedback = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d("filteredFeedback", "",
                                                                    registry, alphaFeedBack, feedbackPart);
   private final double controlDT;
   private final DoubleYoVariable captureKpParallelToMotion = new DoubleYoVariable("captureKpParallel", registry);
   private final DoubleYoVariable captureKpOrthogonalToMotion = new DoubleYoVariable("captureKpOrthogonal", registry);
   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   public ICPProportionalController(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      filteredFeedback.reset();
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
      icpError.set(capturePoint);
      icpError.sub(desiredCapturePoint);

      icpError.getFrameVector2d(tempControl);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredCapturePointVelocity);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getDoubleValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getDoubleValue());
         tempControl.changeFrame(desiredCMP.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getDoubleValue());
      }

      feedbackPart.set(tempControl);
      filteredFeedback.update();
      filteredFeedback.getFrameVector2d(tempControl);
      desiredCMP.add(tempControl);

      return desiredCMP;
   }

   public void setGains(double captureKpParallelToMotion, double captureKpOrthogonalToMotion, double filterBreakFrequencyHertz)
   {
      this.captureKpParallelToMotion.set(captureKpParallelToMotion);
      this.captureKpOrthogonalToMotion.set(captureKpOrthogonalToMotion);
      this.alphaFeedBack.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequencyHertz, controlDT));
   }

   public class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3d x = new Vector3d();
      private final Vector3d y = new Vector3d();
      private final Vector3d z = new Vector3d();
      private final Matrix3d rotation = new Matrix3d();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
      {
         this.xAxis.setAndChangeFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      public void updateTransformToParent(Transform3D transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumn(0, x);
         rotation.setColumn(1, y);
         rotation.setColumn(2, z);

         transformToParent.set(rotation);
      }
   }
}
