package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounterWithColor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

import java.util.concurrent.atomic.AtomicLong;

public class RDXZEDShapePointCounterWithColorDemo
{
   private static final String SVO_FILE = "/home/duncan/Downloads/20260331_165223_AlexTennisBallPickAndPlace2.svo2";

   private static final String[] SHAPES = new String[] {"Sphere", "Capsule"};
   private static final Color DEFAULT_COLOR = new Color(0.45f, 0.75f, 1.0f, 1.0f);

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ZEDSVOPlaybackSensor zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");

   private final ImInt shape = new ImInt(1);
   private final ImFloat shapeRadius = new ImFloat(0.5f);
   private final ImFloat shapeLength = new ImFloat(0.5f);
   private RDXPose3DGizmo shapePoseGizmo;
   private final Color shapeColor = new Color(0.0f, 0.0f, 1.0f, 0.5f);
   private ModelInstance shapeModel;

   private final CUDAShapePointCounterWithColor shapePointCounter = new CUDAShapePointCounterWithColor();
   private final AtomicLong pointsInShape = new AtomicLong();
   private float averageRed = 0.0f;
   private float averageGreen = 0.0f;
   private float averageBlue = 0.0f;

   private final ImGuiMovingPlot pointsPlot = new ImGuiMovingPlot("Points in Sphere", 1000, 300, 200);
   private final ImPlotPlot averageRGBPlot = new ImPlotPlot(140);
   private final ImPlotDoublePlotLine averageRedPlotLine = new ImPlotDoublePlotLine("Average R");
   private final ImPlotDoublePlotLine averageGreenPlotLine = new ImPlotDoublePlotLine("Average G");
   private final ImPlotDoublePlotLine averageBluePlotLine = new ImPlotDoublePlotLine("Average B");
   private final ImBoolean play = new ImBoolean(false);
   private final ImInt requestedPosition = new ImInt();
   private final ImInt currentPosition = new ImInt();
   private final ImInt zedLength = new ImInt();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RawImage latestDepth = null;
   private RawImage latestColor = null;
   private int lastComputedSVOPosition = Integer.MIN_VALUE;
   private int lastComputedShapeType = -1;
   private float lastComputedRadius = Float.NaN;
   private float lastComputedLength = Float.NaN;
   private final Pose3D lastComputedShapePose = new Pose3D();
   private boolean hasLastComputedShapePose = false;

   public RDXZEDShapePointCounterWithColorDemo()
   {
      averageRGBPlot.getPlotLines().add(averageRedPlotLine);
      averageRGBPlot.getPlotLines().add(averageGreenPlotLine);
      averageRGBPlot.getPlotLines().add(averageBluePlotLine);
      averageRGBPlot.clearYFlag(ImPlotAxisFlags.AutoFit);
      averageRGBPlot.setCustomBeforePlotLogic(() -> ImPlot.setNextPlotLimitsY(0.0, 255.0, ImGuiCond.Always));
      averageRedPlotLine.setColor(Color.RED.toIntBits());
      averageGreenPlotLine.setColor(Color.GREEN.toIntBits());
      averageBluePlotLine.setColor(Color.BLUE.toIntBits());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            zedSensor.startSensor();
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            updateShapeModel();
            shapePoseGizmo = new RDXPose3DGizmo();
            shapePoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getImGuiPanelManager().addPanel("Shape Point Counter With Color", RDXZEDShapePointCounterWithColorDemo.this::renderImGuiWidgets);
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            LibGDXTools.toLibGDX(shapePoseGizmo.getTransformToParent(), shapeModel.transform);
            updateShape();
            pointCloudVisualizer.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            zedSensor.close();
            pointCloudVisualizer.destroy();
            shapePoseGizmo.destroyDefault(baseUI.getPrimary3DPanel());
            if (latestDepth != null)
               latestDepth.release();
            if (latestColor != null)
               latestColor.release();
            shapePointCounter.close();
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      pointCloudVisualizer.renderImGuiWidgets();
      ImGui.separator();
      ImGui.pushFont(ImGuiTools.getBigFont());
      ImGui.text("Points in shape: " + pointsInShape.get());
      ImGui.popFont();
      ImGui.text(String.format("Average RGB: %.1f, %.1f, %.1f", averageRed, averageGreen, averageBlue));
      pointsPlot.calculate(pointsInShape.get());
      averageRGBPlot.render();
      ImGui.separator();

      currentPosition.set(zedSensor.getCurrentPosition());
      zedLength.set(zedSensor.getLength());

      ImGui.text("SVO: " + zedSensor.getSVOFileName());
      if (ImGui.checkbox(labels.get("ZED Playback"), play))
      {
         if (play.get())
            zedSensor.play();
         else
            zedSensor.pause();
      }

      ImGui.beginDisabled(play.get());
      if (play.get())
         requestedPosition.set(currentPosition.get());
      if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max(zedLength.get(), 0)))
      {
         zedSensor.setCurrentPosition(requestedPosition.get());
         zedSensor.grabAndNotify();
      }
      ImGui.endDisabled();

      ImGui.text("Frame: " + currentPosition.get() + " / " + Math.max(zedLength.get(), 0));

      ImGui.separator();

      boolean updateModel = ImGui.combo("Shape", shape, SHAPES);
      updateModel |= ImGuiTools.volatileInputFloat("Radius", shapeRadius, 0.01f, 0.1f);
      updateModel |= ImGuiTools.volatileInputFloat("Length", shapeLength, 0.01f, 0.1f);

      if (updateModel)
         updateShapeModel();
   }

   private void updateShapeModel()
   {
      if (shapeModel != null)
         baseUI.getPrimaryScene().removeRenderable(shapeModel);

      float radius = shapeRadius.get();
      float length = shapeLength.get();

      if (shape.get() == 0)
         shapeModel = RDXModelBuilder.createSphere(radius, DEFAULT_COLOR);
      else
      {
         shapeModel = RDXModelBuilder.buildModelInstance(builder ->
         {
            builder.addCylinder(length, radius, new Vector3D(0.0, 0.0, -0.5 * length), DEFAULT_COLOR);
            builder.addSphere(radius, new Vector3D(0.0, 0.0, 0.5 * length), DEFAULT_COLOR);
            builder.addSphere(radius, new Vector3D(0.0, 0.0, -0.5 * length), DEFAULT_COLOR);
         });
      }
      LibGDXTools.setOpacity(shapeModel, 0.5f);

      baseUI.getPrimaryScene().addRenderableProvider(shapeModel, shapeModel, RDXSceneLevel.MODEL);
   }

   private void updateShape()
   {
      long count = 0;
      int currentSVOPosition = zedSensor.getCurrentPosition();
      Pose3D currentShapePose = new Pose3D(shapePoseGizmo.getTransformToParent());
      boolean shapeChanged = shape.get() != lastComputedShapeType
                             || Math.abs(shapeRadius.get() - lastComputedRadius) > 1.0e-6f
                             || Math.abs(shapeLength.get() - lastComputedLength) > 1.0e-6f
                             || !hasLastComputedShapePose
                             || !currentShapePose.epsilonEquals(lastComputedShapePose, 1.0e-6);
      boolean frameChanged = currentSVOPosition != lastComputedSVOPosition;
      boolean shouldRecompute = frameChanged || shapeChanged;

      if (!shouldRecompute)
      {
         count = pointsInShape.get();
         averageRedPlotLine.addValue(averageRed);
         averageGreenPlotLine.addValue(averageGreen);
         averageBluePlotLine.addValue(averageBlue);

         float t = Math.min(Math.max(count / 20000.0f, 0.0f), 1.0f);
         shapeColor.set(t, 0.0f, 1.0f - t, 0.5f);
         LibGDXTools.setDiffuseColor(shapeModel, shapeColor);
         return;
      }

      if (latestDepth != null && latestColor != null)
      {
         count = switch (shape.get())
         {
            case 0 -> shapePointCounter.countPointsInSphere(latestDepth, latestColor, shapePoseGizmo.getTransformToParent().getTranslation(), shapeRadius.get());
            case 1 ->
            {
               Pose3D pose = new Pose3D(shapePoseGizmo.getTransformToParent());
               pose.appendTranslation(0.0, 0.0, shapeLength.get() / 2);
               Point3D32 pointA = new Point3D32(pose.getPosition());
               pose.appendTranslation(0.0, 0.0, -shapeLength.get());
               Point3D32 pointB = new Point3D32(pose.getPosition());
               yield shapePointCounter.countPointsInCapsule(latestDepth, latestColor, pointA, pointB, shapeRadius.get());
            }
            default -> 0;
         };

         pointsInShape.set(count);
         averageRed = shapePointCounter.getAverageRed();
         averageGreen = shapePointCounter.getAverageGreen();
         averageBlue = shapePointCounter.getAverageBlue();
         lastComputedSVOPosition = currentSVOPosition;
         lastComputedShapeType = shape.get();
         lastComputedRadius = shapeRadius.get();
         lastComputedLength = shapeLength.get();
         lastComputedShapePose.set(currentShapePose);
         hasLastComputedShapePose = true;
      }
      else
      {
         pointsInShape.set(0);
         averageRed = 0.0f;
         averageGreen = 0.0f;
         averageBlue = 0.0f;
         lastComputedSVOPosition = currentSVOPosition;
         lastComputedShapeType = shape.get();
         lastComputedRadius = shapeRadius.get();
         lastComputedLength = shapeLength.get();
         lastComputedShapePose.set(currentShapePose);
         hasLastComputedShapePose = true;
      }

      averageRedPlotLine.addValue(averageRed);
      averageGreenPlotLine.addValue(averageGreen);
      averageBluePlotLine.addValue(averageBlue);

      float t = Math.min(Math.max(count / 20000.0f, 0.0f), 1.0f);
      shapeColor.set(t, 0.0f, 1.0f - t, 0.5f);
      LibGDXTools.setDiffuseColor(shapeModel, shapeColor);
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedSensor.waitForGrab();

      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      if (colorImage != null)
      {
         if (latestColor != null)
            latestColor.release();
         latestColor = colorImage.get();

         pointCloudVisualizer.setColorImage(colorImage);
         colorImage.release();
      }
      if (depthImage != null)
      {
         if (latestDepth != null)
            latestDepth.release();
         latestDepth = depthImage.get();

         pointCloudVisualizer.setDepthImage(depthImage);
         depthImage.release();
      }
   }

   public static void main(String[] args)
   {
      new RDXZEDShapePointCounterWithColorDemo();
   }
}
