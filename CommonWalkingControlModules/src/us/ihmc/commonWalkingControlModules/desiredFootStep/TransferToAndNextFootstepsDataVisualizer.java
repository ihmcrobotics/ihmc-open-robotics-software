package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicYoFramePolygon;

public class TransferToAndNextFootstepsDataVisualizer
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoGraphicCoordinateSystem transferFromCoordinateSystem, transferToCoordinateSystem, nextStepCoordinateSystem,
           nextNextStepCoordinateSystem;
   private final YoFrameConvexPolygon2d transferFromPolygon, transferToPolygon, nextStepPolygon, nextNextStepPolygon;
   private final DynamicGraphicYoFramePolygon transferFromPolygonViz, transferToPolygonViz, nextStepPolygonViz, nextNextStepPolygonViz;

   public TransferToAndNextFootstepsDataVisualizer(YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("TransferToAndNextFootstepsDataVisualizer");

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

      transferFromPolygonViz = new DynamicGraphicYoFramePolygon("transferFromPolygon2", transferFromPolygon, "transferFromPolygon2", "", registry,
              polygonVizScale, transferFromPolygonAppearance);
      transferToPolygonViz = new DynamicGraphicYoFramePolygon("transferToPolygon2", transferToPolygon, "transferToPolygon2", "", registry, polygonVizScale,
              transferToPolygonAppearance);
      nextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextStepPolygon2", nextStepPolygon, "nextStepPolygon2", "", registry, polygonVizScale,
              nextFootstepPolygonAppearance);
      nextNextStepPolygonViz = new DynamicGraphicYoFramePolygon("nextNextStepPolygon2", nextNextStepPolygon, "nextNextStepPolygon2", "", registry,
              polygonVizScale, nextNextFootstepPolygonAppearance);

      transferFromPolygonViz.setPosition(0.0, 0.0, 0.001);
      transferToPolygonViz.setPosition(0.0, 0.0, 0.001);
      nextStepPolygonViz.setPosition(0.0, 0.0, 0.001);
      nextNextStepPolygonViz.setPosition(0.0, 0.0, 0.001);

      dynamicGraphicObjectsList.add(transferFromPolygonViz);
      dynamicGraphicObjectsList.add(transferToPolygonViz);
      dynamicGraphicObjectsList.add(nextStepPolygonViz);
      dynamicGraphicObjectsList.add(nextNextStepPolygonViz);

      transferFromCoordinateSystem = new YoGraphicCoordinateSystem("transferFromPose", "", registry, 0.2);
      transferToCoordinateSystem = new YoGraphicCoordinateSystem("transferToPose", "", registry, 0.2);
      nextStepCoordinateSystem = new YoGraphicCoordinateSystem("nextStepPose", "", registry, 0.2);
      nextNextStepCoordinateSystem = new YoGraphicCoordinateSystem("nextNextStepPose", "", registry, 0.2);

      dynamicGraphicObjectsList.add(transferFromCoordinateSystem);
      dynamicGraphicObjectsList.add(transferToCoordinateSystem);
      dynamicGraphicObjectsList.add(nextStepCoordinateSystem);
      dynamicGraphicObjectsList.add(nextNextStepCoordinateSystem);

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
   }

   public void visualizeFootsteps(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      Footstep transferFromFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();
      visualizeFootstep(transferFromFootstep, transferFromPolygon, transferFromPolygonViz, transferFromCoordinateSystem);

      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      visualizeFootstep(transferToFootstep, transferToPolygon, transferToPolygonViz, transferToCoordinateSystem);

      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      visualizeFootstep(nextFootstep, nextStepPolygon, nextStepPolygonViz, nextStepCoordinateSystem);

      Footstep nextNextFootstep = transferToAndNextFootstepsData.getNextNextFootstep();
      visualizeFootstep(nextNextFootstep, nextNextStepPolygon, nextNextStepPolygonViz, nextNextStepCoordinateSystem);
   }

   private static void visualizeFootstep(Footstep footstep, YoFrameConvexPolygon2d footstepPolygon, DynamicGraphicYoFramePolygon footstepPolygonViz,
           YoGraphicCoordinateSystem footstepCoordinateSystem)
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
}
