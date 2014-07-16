package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsDataVisualizer;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;

public class LookaheadFinalDesiredICPCalculator implements FinalDesiredICPCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private boolean VISUALIZE = true;

   private final EnumYoVariable<DesiredICPCalculatorMethod> desiredICPCalculatorMethod = new EnumYoVariable<DesiredICPCalculatorMethod>(
         "desiredICPCalculatorMethod", registry, DesiredICPCalculatorMethod.class);
   private final DoubleYoVariable icpDistanceAlongCentroidSegment = new DoubleYoVariable("icpDistanceAlongCentroidSegment", registry);
   private final DoubleYoVariable w0ForICPGenerator = new DoubleYoVariable("w0ForICPGenerator", registry);
   private final DoubleYoVariable estimatedStepTimeForICPGenerator = new DoubleYoVariable("estimatedStepTimeForICPGenerator", registry);
   private final DoubleYoVariable scaleFactorForICPGenerator = new DoubleYoVariable("scaleFactorForICPGenerator", registry);

   private final TransferToAndNextFootstepsDataVisualizer transferToAndNextFootstepsDataVisualizer;
   private final DynamicGraphicPosition finalDesiredICPGraphicPosition;

   private final ShiftInsideFinalDesiredICPCalculator shiftInsideFinalDesiredICPCalculator = new ShiftInsideFinalDesiredICPCalculator(registry, 0.0, 0.04);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public LookaheadFinalDesiredICPCalculator(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (dynamicGraphicObjectsListRegistry == null)
         VISUALIZE = false;

      if (VISUALIZE)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FinalDesiredICPCalculator");
         transferToAndNextFootstepsDataVisualizer = new TransferToAndNextFootstepsDataVisualizer(registry, dynamicGraphicObjectsListRegistry);

         double finalDesiredGraphicScale = 0.005;
         finalDesiredICPGraphicPosition = new DynamicGraphicPosition("finalDesiredICP", "", registry, finalDesiredGraphicScale, YoAppearance.Yellow(),
               GraphicType.CROSS);
         dynamicGraphicObjectsList.add(finalDesiredICPGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerArtifact("FinalDesiredICP", finalDesiredICPGraphicPosition.createArtifact());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      }

      else
      {
         transferToAndNextFootstepsDataVisualizer = null;
         finalDesiredICPGraphicPosition = null;
      }

      desiredICPCalculatorMethod.set(DesiredICPCalculatorMethod.SHIFT_INSIDE);
      icpDistanceAlongCentroidSegment.set(0.04);

      parentRegistry.addChild(registry);
   }

   public void setMethod(DesiredICPCalculatorMethod method)
   {
      desiredICPCalculatorMethod.set(method);
   }

   private FramePoint2d finalDesiredICP;

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      finalDesiredICP = getFinalDesiredICPForWalking(transferToAndNextFootstepsData);
   }

   public FramePoint2d getFinalDesiredICP()
   {
      return finalDesiredICP;
   }

   private FramePoint2d getFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      switch (desiredICPCalculatorMethod.getEnumValue())
      {
      case SHIFT_INSIDE:
      {
         shiftInsideFinalDesiredICPCalculator.initialize(transferToAndNextFootstepsData);
         return shiftInsideFinalDesiredICPCalculator.getFinalDesiredICP();
      }

      case CENTROID_TO_CENTROID:
      {
         return getFinalDesiredICPCentroidToCentroid(transferToAndNextFootstepsData);
      }

      case CENTROID_TO_CENTROID_WITH_CALCULATED_SCALAR:
      {
         return getFinalDesiredICPCentroidToCentroidWithCalculatedScalar(transferToAndNextFootstepsData);
      }

      default:
      {
         throw new RuntimeException("Should not get here!");
      }
      }
   }

   private FramePoint2d getFinalDesiredICPCentroidToCentroid(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (transferToAndNextFootstepsDataVisualizer != null)
      {
         transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
      }

      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

      FrameConvexPolygon2d transferToFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(transferToFootstep, worldFrame);

      FramePoint2d transferToCentroid = transferToFootPolygon.getCentroidCopy();

      if (nextFootstep == null)
      {
         return transferToCentroid;
      }

      FrameConvexPolygon2d nextFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(nextFootstep, worldFrame);
      FramePoint2d nextCentroid = nextFootPolygon.getCentroidCopy();

      FrameVector2d vectorFromTransferToNext = new FrameVector2d(nextCentroid);
      vectorFromTransferToNext.sub(transferToCentroid);
      vectorFromTransferToNext.normalize();
      vectorFromTransferToNext.scale(icpDistanceAlongCentroidSegment.getDoubleValue());

      FramePoint2d finalDesiredICP = new FramePoint2d(transferToCentroid);
      finalDesiredICP.add(vectorFromTransferToNext);

      visualizeFinalDesiredICP(finalDesiredICP);

      return finalDesiredICP;
   }

   private FramePoint2d getFinalDesiredICPCentroidToCentroidWithCalculatedScalar(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (transferToAndNextFootstepsDataVisualizer != null)
      {
         transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
      }

      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

      FrameConvexPolygon2d transferToFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(transferToFootstep, worldFrame);

      FramePoint2d transferToCentroid = transferToFootPolygon.getCentroidCopy();

      FramePoint2d finalDesiredICP;
      if (nextFootstep == null)
      {
         finalDesiredICP = transferToCentroid;
      }
      else
      {
         FrameConvexPolygon2d nextFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(nextFootstep, worldFrame);
         FramePoint2d nextCentroid = nextFootPolygon.getCentroidCopy();

         FrameVector2d vectorFromTransferToNext = new FrameVector2d(nextCentroid);
         vectorFromTransferToNext.sub(transferToCentroid);

         double sf = vectorFromTransferToNext.length();
         double w0 = transferToAndNextFootstepsData.getW0();
         w0ForICPGenerator.set(w0);
         double estimatedStepTime = transferToAndNextFootstepsData.getEstimatedStepTime();
         estimatedStepTimeForICPGenerator.set(estimatedStepTime);
         double s0 = sf / (Math.exp(w0 * estimatedStepTime));
         scaleFactorForICPGenerator.set(s0);

         vectorFromTransferToNext.normalize();
         vectorFromTransferToNext.scale(s0);

         finalDesiredICP = new FramePoint2d(transferToCentroid);
         finalDesiredICP.add(vectorFromTransferToNext);
      }

      visualizeFinalDesiredICP(finalDesiredICP);

      return finalDesiredICP;
   }

   private void visualizeFinalDesiredICP(FramePoint2d finalDesiredICP)
   {
      if (VISUALIZE)
      {
         finalDesiredICP.getReferenceFrame().checkIsWorldFrame();
         finalDesiredICPGraphicPosition.setPosition(finalDesiredICP.getX(), finalDesiredICP.getY(), 0.0);
      }
   }

   public static enum DesiredICPCalculatorMethod
   {
      SHIFT_INSIDE, CENTROID_TO_CENTROID, CENTROID_TO_CENTROID_WITH_CALCULATED_SCALAR;
   }
}