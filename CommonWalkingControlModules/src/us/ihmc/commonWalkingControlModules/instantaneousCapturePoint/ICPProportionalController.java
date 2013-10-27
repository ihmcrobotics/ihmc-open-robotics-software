package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

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
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2d tempControl = new FrameVector2d(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final YoFrameVector2d icpErrorIntegrated = new YoFrameVector2d("icpErrorIntegrated", "", worldFrame, registry);
   
   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);
   
   private final DoubleYoVariable alphaFeedBack = new DoubleYoVariable("alphaFeedBack", registry);
   private final YoFrameVector2d desiredCMPToICP = new YoFrameVector2d("desiredCMPToICP", "", worldFrame, registry);
   private final YoFrameVector2d rawCMPOutput = new YoFrameVector2d("rawCMPOutput", "", worldFrame, registry);
   private final AlphaFilteredYoFrameVector2d filteredCMPOutput = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d("filteredCMPOutput", "",
                                                                    registry, alphaFeedBack, rawCMPOutput);

   private final DoubleYoVariable maxDistanceBetweenICPAndCMP = new DoubleYoVariable("maxDistanceBetweenICPAndCMP", registry);
   
   private final YoFramePoint icpPosition;
   private final DoubleYoVariable alphaICPVelocity;
   private final FilteredVelocityYoFrameVector icpVelocity;
   private final FrameVector2d icpDamping = new FrameVector2d(worldFrame);
   private final FrameVector2d icpIntegral = new FrameVector2d(worldFrame);
   
   private final double controlDT;
   private final DoubleYoVariable captureKpParallelToMotion = new DoubleYoVariable("captureKpParallel", registry);
   private final DoubleYoVariable captureKpOrthogonalToMotion = new DoubleYoVariable("captureKpOrthogonal", registry);

   private final DoubleYoVariable captureKd = new DoubleYoVariable("captureKd", registry);
   private final DoubleYoVariable captureKi = new DoubleYoVariable("captureKi", registry);
   
   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   public ICPProportionalController(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);
      
      icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
      alphaICPVelocity = new DoubleYoVariable("alphaICPVelocity", registry);
      icpVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("icpVelocity", "", alphaICPVelocity, controlDT, parentRegistry, icpPosition);
      parentRegistry.addChild(registry);
      
      maxDistanceBetweenICPAndCMP.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      filteredCMPOutput.reset();
   }

   public FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity,
           double omega0)
   {
      desiredCapturePointVelocity.changeFrame(desiredCapturePoint.getReferenceFrame());
      FramePoint2d desiredCMP = new FramePoint2d(capturePoint);

      icpPosition.set(capturePoint.getX(), capturePoint.getY(), 0.0);
      icpVelocity.update();
      
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

      icpDamping.set(icpVelocity.getX(), icpVelocity.getY());
      icpDamping.scale(captureKd.getDoubleValue());
      double length = icpDamping.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         icpDamping.scale(maxLength/length);
      }
      
      tempControl.add(icpDamping);
      
      icpErrorIntegrated.add(icpError);
      icpErrorIntegrated.getFrameVector2d(icpIntegral);
      icpIntegral.scale(captureKi.getDoubleValue());
      length = icpDamping.length();
      if (length > maxLength)
      {
         icpIntegral.scale(maxLength/length);
      }
      
      tempControl.add(icpIntegral);
      
      feedbackPart.set(tempControl);
      desiredCMP.add(tempControl);

      desiredCMPToICP.sub(capturePoint, desiredCMP);

      double distanceDesiredCMPToICP = desiredCMPToICP.length();
      if (distanceDesiredCMPToICP > maxDistanceBetweenICPAndCMP.getDoubleValue())
      {
         desiredCMPToICP.scale(maxDistanceBetweenICPAndCMP.getDoubleValue() / distanceDesiredCMPToICP);
         desiredCMP.set(capturePoint);
         desiredCMP.sub(desiredCMPToICP.getFrameVector2dCopy());
      }
      
      rawCMPOutput.set(desiredCMP);
      
      filteredCMPOutput.update();
      filteredCMPOutput.getFramePoint2d(desiredCMP);
      
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
