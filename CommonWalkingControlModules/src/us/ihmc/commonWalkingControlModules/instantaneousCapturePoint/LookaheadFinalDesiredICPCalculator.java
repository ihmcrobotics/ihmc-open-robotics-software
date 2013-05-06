package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicYoFramePolygon;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class LookaheadFinalDesiredICPCalculator implements FinalDesiredICPCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private boolean VISUALIZE = true;

   private final EnumYoVariable<DesiredICPCalculatorMethod> desiredICPCalculatorMethod =
      new EnumYoVariable<DesiredICPCalculatorMethod>("desiredICPCalculatorMethod", registry, DesiredICPCalculatorMethod.class);
   private final DoubleYoVariable icpDistanceAlongCentroidSegment = new DoubleYoVariable("icpDistanceAlongCentroidSegment", registry);
   private final DoubleYoVariable w0ForICPGenerator = new DoubleYoVariable("w0ForICPGenerator", registry);
   private final DoubleYoVariable estimatedStepTimeForICPGenerator = new DoubleYoVariable("estimatedStepTimeForICPGenerator", registry);
   private final DoubleYoVariable scaleFactorForICPGenerator = new DoubleYoVariable("scaleFactorForICPGenerator", registry);

   private final DynamicGraphicPosition finalDesiredICPGraphicPosition;
   private final DynamicGraphicCoordinateSystem transferFromCoordinateSystem, transferToCoordinateSystem, nextStepCoordinateSystem, nextNextStepCoordinateSystem;

   private final YoFrameConvexPolygon2d transferFromPolygon, transferToPolygon, nextStepPolygon, nextNextStepPolygon;
   private final DynamicGraphicYoFramePolygon transferFromPolygonViz, transferToPolygonViz, nextStepPolygonViz, nextNextStepPolygonViz;
   
   
   private final ShiftInsideFinalDesiredICPCalculator shiftInsideFinalDesiredICPCalculator = new ShiftInsideFinalDesiredICPCalculator(registry, 0.0, 0.04);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public LookaheadFinalDesiredICPCalculator(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (dynamicGraphicObjectsListRegistry == null)
         VISUALIZE = false;

      if (VISUALIZE)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FinalDesiredICPCalculator");

         double finalDesiredGraphicScale = 0.005;
         finalDesiredICPGraphicPosition = new DynamicGraphicPosition("finalDesiredICP", "", registry, finalDesiredGraphicScale, YoAppearance.Yellow(),
                 GraphicType.CROSS);
         dynamicGraphicObjectsList.add(finalDesiredICPGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerArtifact("FinalDesiredICP", finalDesiredICPGraphicPosition.createArtifact());

         int maxNumberOfVertices = 6;
         transferFromPolygon = new YoFrameConvexPolygon2d("transferFromPolygon", "", worldFrame, maxNumberOfVertices, registry);
         transferToPolygon = new YoFrameConvexPolygon2d("transferToPolygon", "", worldFrame, maxNumberOfVertices, registry);
         nextStepPolygon = new YoFrameConvexPolygon2d("nextStepPolygon", "", worldFrame, maxNumberOfVertices, registry);
         nextNextStepPolygon = new YoFrameConvexPolygon2d("nextNextStepPolygon", "", worldFrame, maxNumberOfVertices, registry);

         double polygonVizScale = 1.0;

         // ooh, colors!
         AppearanceDefinition transferFromPolygonAppearance = YoAppearance.Gold();
         YoAppearance.makeTransparent(transferFromPolygonAppearance, 0.5);
         AppearanceDefinition transferToPolygonAppearance = YoAppearance.MidnightBlue();
         YoAppearance.makeTransparent(transferToPolygonAppearance, 0.5);
         AppearanceDefinition nextFootstepPolygonAppearance = YoAppearance.IndianRed();
         YoAppearance.makeTransparent(nextFootstepPolygonAppearance, 0.5);
         AppearanceDefinition nextNextFootstepPolygonAppearance = YoAppearance.Lavender();

         transferFromPolygonViz = new DynamicGraphicYoFramePolygon("transferFromPolygon", transferFromPolygon, "transferFromPolygon", "",
               registry, polygonVizScale, transferFromPolygonAppearance);
         transferToPolygonViz = new DynamicGraphicYoFramePolygon("transferToPolygon", transferToPolygon, "transferToPolygon", "",
                                                                registry, polygonVizScale, transferToPolygonAppearance);
         nextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextStepPolygon", nextStepPolygon, "nextStepPolygon", "",
                                                              registry, polygonVizScale, nextFootstepPolygonAppearance);
         nextNextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextNextStepPolygon", nextNextStepPolygon,
                                                                  "nextNextStepPolygon", "", registry, polygonVizScale, nextNextFootstepPolygonAppearance);

         transferFromPolygonViz.setPosition(0.0, 0.0, 0.001);
         transferToPolygonViz.setPosition(0.0, 0.0, 0.001);
         nextStepPolygonViz.setPosition(0.0, 0.0, 0.001);
         nextNextStepPolygonViz.setPosition(0.0, 0.0, 0.001);

         dynamicGraphicObjectsList.add(transferFromPolygonViz);
         dynamicGraphicObjectsList.add(transferToPolygonViz);
         dynamicGraphicObjectsList.add(nextStepPolygonViz);
         dynamicGraphicObjectsList.add(nextNextStepPolygonViz);

         transferFromCoordinateSystem = new DynamicGraphicCoordinateSystem("transferFromPose", "", registry, 0.2);
         transferToCoordinateSystem = new DynamicGraphicCoordinateSystem("transferToPose", "", registry, 0.2);
         nextStepCoordinateSystem = new DynamicGraphicCoordinateSystem("nextStepPose", "", registry, 0.2);
         nextNextStepCoordinateSystem = new DynamicGraphicCoordinateSystem("nextNextStepPose", "", registry, 0.2);

         dynamicGraphicObjectsList.add(transferFromCoordinateSystem);
         dynamicGraphicObjectsList.add(transferToCoordinateSystem);
         dynamicGraphicObjectsList.add(nextStepCoordinateSystem);
         dynamicGraphicObjectsList.add(nextNextStepCoordinateSystem);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      }

      else
      {
         transferFromPolygon = null;
         transferToPolygon = null;
         nextStepPolygon = null;
         nextNextStepPolygon = null;

         transferFromPolygonViz = null;
         transferToPolygonViz = null;
         nextStepPolygonViz = null;
         nextNextStepPolygonViz = null;
         
         transferFromCoordinateSystem = null;
         transferToCoordinateSystem = null;
         nextStepCoordinateSystem = null;
         nextNextStepCoordinateSystem = null;

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

   public FramePoint2d getFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      switch (desiredICPCalculatorMethod.getEnumValue())
      {
         case SHIFT_INSIDE :
         {
            return shiftInsideFinalDesiredICPCalculator.getFinalDesiredICPForWalking(transferToAndNextFootstepsData);
         }

         case CENTROID_TO_CENTROID :
         {
            return getFinalDesiredICPCentroidToCentroid(transferToAndNextFootstepsData);
         }

         case CENTROID_TO_CENTROID_WITH_CALCULATED_SCALAR :
         {
            return getFinalDesiredICPCentroidToCentroidWithCalculatedScalar(transferToAndNextFootstepsData);
         }

         default :
         {
            throw new RuntimeException("Should not get here!");
         }
      }
   }

   private FramePoint2d getFinalDesiredICPCentroidToCentroid(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      visualizeFootsteps(transferToAndNextFootstepsData);

      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

//    Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();

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
      visualizeFootsteps(transferToAndNextFootstepsData);

      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

//    Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();

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

   public void visualizeFootsteps(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (!VISUALIZE)
         return;

      Footstep transferFromFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();
      visualizeFootstep(transferFromFootstep, transferFromPolygon, transferFromPolygonViz, transferFromCoordinateSystem);
      
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      visualizeFootstep(transferToFootstep, transferToPolygon, transferToPolygonViz, transferToCoordinateSystem);

      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      visualizeFootstep(nextFootstep, nextStepPolygon, nextStepPolygonViz, nextStepCoordinateSystem);
 
      Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();
      visualizeFootstep(nextNextFootstep, nextNextStepPolygon, nextNextStepPolygonViz, nextNextStepCoordinateSystem);
   }

   private static void visualizeFootstep(Footstep footstep, YoFrameConvexPolygon2d footstepPolygon, DynamicGraphicYoFramePolygon footstepPolygonViz, DynamicGraphicCoordinateSystem footstepCoordinateSystem)
   {
      if (footstep != null)
      {
         FrameConvexPolygon2d nextFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(footstep, footstep.getSoleReferenceFrame());
         footstepPolygon.setConvexPolygon2d(nextFootPolygon.getConvexPolygon2d());
         
         footstepPolygonViz.setToReferenceFrame(footstep.getSoleReferenceFrame());
         footstepCoordinateSystem.setToReferenceFrame(footstep.getPoseReferenceFrame());
      }
      else
      {
         footstepPolygon.hide();
         footstepCoordinateSystem.hide();
      }
   }

   public static enum DesiredICPCalculatorMethod {SHIFT_INSIDE, CENTROID_TO_CENTROID, CENTROID_TO_CENTROID_WITH_CALCULATED_SCALAR;}
}
