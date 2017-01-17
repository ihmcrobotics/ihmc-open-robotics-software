package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JCheckBox;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;


public class TrackCheckBox extends JCheckBox implements ActionListener
{
   private static final String TRACK = "Track";
   private static final long serialVersionUID = -9012772363648771937L;
   private ActiveCameraHolder cameraHolder;

   public TrackCheckBox(ActiveCameraHolder cameraHolder)
   {
      super(TRACK);
      this.setName(TRACK);

      this.cameraHolder = cameraHolder;

      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      this.setSelected(camera.isTracking());
      addActionListener(this);
   }

   public void actionPerformed(ActionEvent actionEvent)
   {
      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      camera.setTracking(this.isSelected());

   }

   public void makeCameraConsistent()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         public void run()
         {
            CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
            camera.setTracking(isSelected());
         }});
   }

   public void makeCheckBoxConsistent()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         public void run()
         {
            CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
            if (camera != null) setSelected(camera.isTracking());
         }});
   }
}
