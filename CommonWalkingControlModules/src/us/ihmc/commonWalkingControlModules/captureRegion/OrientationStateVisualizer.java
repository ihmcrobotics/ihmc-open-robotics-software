package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;

public class OrientationStateVisualizer
{
   private static final Color REDUCED_SUPPORT_POLYGON_COLOR = Color.ORANGE;
   private static final Color PELVIS_X_AXIS_COLOR = Color.RED;
   private static final Color PELVIS_Y_AXIS_COLOR = Color.WHITE;
   private static final double PELVIS_AXES_LENGTH = 0.05;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name + "Registry");

   private final ReferenceFrame pelvisZUpFrame;

   private final YoArtifactPolygon reducedSupportPolygonArtifact;
   private YoFrameConvexPolygon2d yoReducedSupportPolygon;
   private FrameConvexPolygon2d reducedSupportPolygon;

   private final YoFrameLineSegment2d yoPelvisXAxisLineSegment;
   private final YoFrameLineSegment2d yoPelvisYAxisLineSegment;

   private final FramePoint2d pelvisPosition2d = new FramePoint2d();
   private final FrameVector tempVector = new FrameVector();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   public OrientationStateVisualizer(ReferenceFrame pelvisZUpFrame, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.pelvisZUpFrame = pelvisZUpFrame;
      String reducedSupportPolygonCaption = "ReducedSupportPolygon";
      yoReducedSupportPolygon = new YoFrameConvexPolygon2d(reducedSupportPolygonCaption, "", worldFrame, 8, registry);
      reducedSupportPolygon = new FrameConvexPolygon2d(worldFrame);
      reducedSupportPolygonArtifact = new YoArtifactPolygon(reducedSupportPolygonCaption, yoReducedSupportPolygon, REDUCED_SUPPORT_POLYGON_COLOR, false);

      yoPelvisXAxisLineSegment = new YoFrameLineSegment2d("pelvisXAxis", "", worldFrame, registry);
      yoPelvisYAxisLineSegment = new YoFrameLineSegment2d("pelvisYAxis", "", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifact(reducedSupportPolygonCaption, reducedSupportPolygonArtifact);
         yoGraphicsListRegistry.registerArtifact("PelvisCoordinate", new YoArtifactLineSegment2d("Pelvis X Axis", yoPelvisXAxisLineSegment, PELVIS_X_AXIS_COLOR, 0.007, 0.01));
         yoGraphicsListRegistry.registerArtifact("PelvisCoordinate", new YoArtifactLineSegment2d("Pelvis Y Axis", yoPelvisYAxisLineSegment, PELVIS_Y_AXIS_COLOR, 0.007, 0.01));
      }

      parentRegistry.addChild(registry);
   }

   public void updateReducedSupportPolygon(FrameConvexPolygon2d newReducedSupportPolygon)
   {
      reducedSupportPolygon.setIncludingFrameAndUpdate(newReducedSupportPolygon);
      reducedSupportPolygon.changeFrame(worldFrame);

      try
      {
         yoReducedSupportPolygon.setFrameConvexPolygon2d(reducedSupportPolygon);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void hideReducedSupportPolygon()
   {
      yoReducedSupportPolygon.hide();
   }

   public void updatePelvisVisualization()
   {
      pelvisPosition2d.setToZero(pelvisZUpFrame);
      pelvisPosition2d.changeFrameAndProjectToXYPlane(worldFrame);

      tempVector.setIncludingFrame(pelvisZUpFrame, PELVIS_AXES_LENGTH, 0.0, 0.0);
      tempVector.changeFrame(worldFrame);
      tempVector2d.setByProjectionOntoXYPlane(tempVector);
      yoPelvisXAxisLineSegment.set(pelvisPosition2d, tempVector2d);

      tempVector.setIncludingFrame(pelvisZUpFrame, 0.0, PELVIS_AXES_LENGTH, 0.0);
      tempVector.changeFrame(worldFrame);
      tempVector2d.setByProjectionOntoXYPlane(tempVector);
      yoPelvisYAxisLineSegment.set(pelvisPosition2d, tempVector2d);
   }
}
