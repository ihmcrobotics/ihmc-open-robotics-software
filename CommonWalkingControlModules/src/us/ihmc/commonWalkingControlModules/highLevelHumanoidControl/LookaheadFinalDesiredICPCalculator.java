package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
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
   
   private final EnumYoVariable<DesiredICPCalculatorMethod> desiredICPCalculatorMethod = new EnumYoVariable<DesiredICPCalculatorMethod>("desiredICPCalculatorMethod", registry, DesiredICPCalculatorMethod.class);
   private final DoubleYoVariable icpDistanceAlongCentroidSegment = new DoubleYoVariable("icpDistanceAlongCentroidSegment", registry);
   
   private final DynamicGraphicPosition finalDesiredICPGraphicPosition;
   private final DynamicGraphicCoordinateSystem transferToCoordinateSystem, nextStepCoordinateSystem, nextNextStepCoordinateSystem;
   
   private final YoFrameConvexPolygon2d transferToPolygon, nextStepPolygon, nextNextStepPolygon;
      
   private final ShiftInsideFinalDesiredICPCalculator shiftInsideFinalDesiredICPCalculator = new ShiftInsideFinalDesiredICPCalculator(registry);
   
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public LookaheadFinalDesiredICPCalculator(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (dynamicGraphicObjectsListRegistry == null) VISUALIZE = false;

      if (VISUALIZE)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FinalDesiredICPCalculator");

         double finalDesiredGraphicScale = 0.05;
         finalDesiredICPGraphicPosition = new DynamicGraphicPosition("finalDesiredICP", "", registry, finalDesiredGraphicScale, YoAppearance.Yellow(), GraphicType.CROSS);
         dynamicGraphicObjectsList.add(finalDesiredICPGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerArtifact("FinalDesiredICP", finalDesiredICPGraphicPosition.createArtifact());

         int maxNumberOfVertices = 6;
         transferToPolygon = new YoFrameConvexPolygon2d("transferToPolygon", "", worldFrame, maxNumberOfVertices , registry);
         nextStepPolygon = new YoFrameConvexPolygon2d("nextStepPolygon", "", worldFrame, maxNumberOfVertices , registry);
         nextNextStepPolygon = new YoFrameConvexPolygon2d("nextNextStepPolygon", "", worldFrame, maxNumberOfVertices , registry);

         double polygonVizScale = 1.0;
         DynamicGraphicYoFramePolygon transferToPolygonViz = new DynamicGraphicYoFramePolygon("transferToPolygon", transferToPolygon, "transferToPolygon", "", registry, polygonVizScale, YoAppearance.Black());
         DynamicGraphicYoFramePolygon nextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextStepPolygon", nextStepPolygon, "nextStepPolygon", "", registry, polygonVizScale, YoAppearance.Blue());
         DynamicGraphicYoFramePolygon nextNextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextNextStepPolygon", nextNextStepPolygon, "nextNextStepPolygon", "", registry, polygonVizScale, YoAppearance.Brown());

         transferToPolygonViz.setPosition(0.0, 0.0, 0.001);
         nextStepPolygonViz.setPosition(0.0, 0.0, 0.001);
         nextNextStepPolygonViz.setPosition(0.0, 0.0, 0.001);
         
         dynamicGraphicObjectsList.add(transferToPolygonViz);
         dynamicGraphicObjectsList.add(nextStepPolygonViz);
         dynamicGraphicObjectsList.add(nextNextStepPolygonViz);


         transferToCoordinateSystem = new DynamicGraphicCoordinateSystem("transferToPose", "", registry, 0.2);
         nextStepCoordinateSystem = new DynamicGraphicCoordinateSystem("nextStepPose", "", registry, 0.2);
         nextNextStepCoordinateSystem = new DynamicGraphicCoordinateSystem("nextNextStepPose", "", registry, 0.2);

         dynamicGraphicObjectsList.add(transferToCoordinateSystem);
         dynamicGraphicObjectsList.add(nextStepCoordinateSystem);
         dynamicGraphicObjectsList.add(nextNextStepCoordinateSystem);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);

      }
      
      else
      {
         transferToPolygon = null;
         nextStepPolygon = null;
         nextNextStepPolygon = null;

         transferToCoordinateSystem = null;
         nextStepCoordinateSystem = null;
         nextNextStepCoordinateSystem = null;
         
         finalDesiredICPGraphicPosition = null;
      }

      desiredICPCalculatorMethod.set(DesiredICPCalculatorMethod.SHIFT_INSIDE);
      icpDistanceAlongCentroidSegment.set(0.04);
      
      parentRegistry.addChild(registry);
   }
   
   public FramePoint2d getFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      switch (desiredICPCalculatorMethod.getEnumValue())
      {
      case SHIFT_INSIDE:
      {
         return shiftInsideFinalDesiredICPCalculator.getFinalDesiredICPForWalking(transferToAndNextFootstepsData);
      }
      case CENTROID_TO_CENTROID:
      {
         return getFinalDesiredICPCentroidToCentroid(transferToAndNextFootstepsData);
      }
      default:
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
//      Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();
            
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
      if (!VISUALIZE) return;
      
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();

      FrameConvexPolygon2d transferToFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(transferToFootstep, worldFrame);
      transferToPolygon.setConvexPolygon2d(transferToFootPolygon.getConvexPolygon2d());
      transferToCoordinateSystem.setToReferenceFrame(transferToFootstep.getPoseReferenceFrame());

      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

      if (nextFootstep != null)
      {
         FrameConvexPolygon2d nextFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(nextFootstep, worldFrame);
         nextStepPolygon.setConvexPolygon2d(nextFootPolygon.getConvexPolygon2d());   
         nextStepCoordinateSystem.setToReferenceFrame(nextFootstep.getPoseReferenceFrame());
      }
   }
   

   private static enum DesiredICPCalculatorMethod
   {
         SHIFT_INSIDE, CENTROID_TO_CENTROID;
   }
}
