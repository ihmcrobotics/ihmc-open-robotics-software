package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicPolygon;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class TransferToAndNextFootstepsDataVisualizer
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoGraphicCoordinateSystem transferFromCoordinateSystem, transferToCoordinateSystem, nextStepCoordinateSystem,
           nextNextStepCoordinateSystem;
   private final YoFrameConvexPolygon2d transferFromPolygon, transferToPolygon, nextStepPolygon, nextNextStepPolygon;
   private final YoGraphicPolygon transferFromPolygonViz, transferToPolygonViz, nextStepPolygonViz, nextNextStepPolygonViz;

   public TransferToAndNextFootstepsDataVisualizer(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("TransferToAndNextFootstepsDataVisualizer");

      int maxNumberOfVertices = 8;
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

      transferFromPolygonViz = new YoGraphicPolygon("transferFromPolygon2", transferFromPolygon, "transferFromPolygon2", "", registry,
              polygonVizScale, transferFromPolygonAppearance);
      transferToPolygonViz = new YoGraphicPolygon("transferToPolygon2", transferToPolygon, "transferToPolygon2", "", registry, polygonVizScale,
              transferToPolygonAppearance);
      nextStepPolygonViz = new YoGraphicPolygon("nextStepPolygon2", nextStepPolygon, "nextStepPolygon2", "", registry, polygonVizScale,
              nextFootstepPolygonAppearance);
      nextNextStepPolygonViz = new YoGraphicPolygon("nextNextStepPolygon2", nextNextStepPolygon, "nextNextStepPolygon2", "", registry,
              polygonVizScale, nextNextFootstepPolygonAppearance);

      transferFromPolygonViz.setPosition(0.0, 0.0, 0.001);
      transferToPolygonViz.setPosition(0.0, 0.0, 0.001);
      nextStepPolygonViz.setPosition(0.0, 0.0, 0.001);
      nextNextStepPolygonViz.setPosition(0.0, 0.0, 0.001);

      yoGraphicsList.add(transferFromPolygonViz);
      yoGraphicsList.add(transferToPolygonViz);
      yoGraphicsList.add(nextStepPolygonViz);
      yoGraphicsList.add(nextNextStepPolygonViz);

      transferFromCoordinateSystem = new YoGraphicCoordinateSystem("transferFromPose", "", registry, 0.2);
      transferToCoordinateSystem = new YoGraphicCoordinateSystem("transferToPose", "", registry, 0.2);
      nextStepCoordinateSystem = new YoGraphicCoordinateSystem("nextStepPose", "", registry, 0.2);
      nextNextStepCoordinateSystem = new YoGraphicCoordinateSystem("nextNextStepPose", "", registry, 0.2);

      yoGraphicsList.add(transferFromCoordinateSystem);
      yoGraphicsList.add(transferToCoordinateSystem);
      yoGraphicsList.add(nextStepCoordinateSystem);
      yoGraphicsList.add(nextNextStepCoordinateSystem);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
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

   private static void visualizeFootstep(Footstep footstep, YoFrameConvexPolygon2d footstepPolygon, YoGraphicPolygon footstepPolygonViz,
           YoGraphicCoordinateSystem footstepCoordinateSystem)
   {
      if (footstep != null)
      {
         // FIXME I broke it Sylvain
//         FrameConvexPolygon2d nextFootPolygon = FootstepUtils.getProjectedFootPolygonInFrame(footstep, footstep.getSoleReferenceFrame());
//         footstepPolygon.setConvexPolygon2d(nextFootPolygon.getConvexPolygon2d());

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
