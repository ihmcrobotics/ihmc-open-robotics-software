package us.ihmc.simulationconstructionset.gui;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.ViewportConfiguration;

public class StandardViewSetup
{

   public static void setupStandardViews(StandardSimulationGUI standardSimulationGUI)
   {
      // Camera and Viewport Configurations
      CameraConfiguration camera1 = new CameraConfiguration("Right Side");

      camera1.setCameraFix(0.0, 0.0, 0.6);
      camera1.setCameraPosition(0, -6, 1);
      camera1.setCameraTracking(true, true, true, false);
      camera1.setCameraDolly(false, true, true, true);

      CameraConfiguration camera2 = new CameraConfiguration("Back View");

      camera2.setCameraFix(0.0, 0.0, 0.6);
      camera2.setCameraPosition(-6.0, -7.0, 1.5);
      camera2.setCameraTracking(true, true, true, false);
      camera2.setCameraDolly(false, true, true, true);
      camera2.setCameraDollyOffsets(2, 12.0, 0.0);
      camera2.setCameraTrackingOffsets(0.0, 0.0, 0.0);

      CameraConfiguration camera3 = new CameraConfiguration("Front View");

      camera3.setCameraFix(0, 0.0, 0.6);
      camera3.setCameraPosition(12.5, 1, 1.5);
      camera3.setCameraTracking(true, true, true, false);
      camera3.setCameraDolly(false, true, true, true);
      camera3.setCameraDollyOffsets(2, 12, 0);
      camera3.setCameraTrackingOffsets(0.0, 0.0, 0.0);

      CameraConfiguration camera4 = new CameraConfiguration("Left View");

      camera4.setCameraFix(0, 0.0, 0.6);
      camera4.setCameraPosition(0, 6, 1);
      camera4.setCameraTracking(true, true, true, false);
      camera4.setCameraDolly(false, true, true, true);
      camera4.setCameraDollyOffsets(2, 12, 0);
      camera4.setCameraTrackingOffsets(0.0, 0.0, 0.0);
      
      standardSimulationGUI.setupCamera(camera1);
      standardSimulationGUI.setupCamera(camera2);
      standardSimulationGUI.setupCamera(camera3);
      standardSimulationGUI.setupCamera(camera4);

      ViewportConfiguration view1 = new ViewportConfiguration("Normal View");

      view1.addCameraView("Right Side", 0, 0, 2, 2);

      ViewportConfiguration view2 = new ViewportConfiguration("Split Screen");

      view2.addCameraView("Right Side", 0, 0, 2, 2);
      view2.addCameraView("Back View", 2, 0, 2, 2);

      ViewportConfiguration view3 = new ViewportConfiguration("Three Views");

      view3.addCameraView("Right Side", 0, 0, 2, 2);
      view3.addCameraView("Back View", 2, 0, 2, 2);
      view3.addCameraView("Front View", 0, 2, 4, 2);

      ViewportConfiguration view4 = new ViewportConfiguration("Four Views");

      view4.addCameraView("Right Side", 0, 0, 2, 2);
      view4.addCameraView("Back View", 2, 0, 2, 2);
      view4.addCameraView("Front View", 0, 2, 2, 2);
      view4.addCameraView("Left View", 2, 2, 2, 2);

      standardSimulationGUI.setupViewport(view1);
      standardSimulationGUI.setupViewport(view2);
      standardSimulationGUI.setupViewport(view3);
      standardSimulationGUI.setupViewport(view4);
   }

}
