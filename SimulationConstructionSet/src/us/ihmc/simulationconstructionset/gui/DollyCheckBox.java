package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JCheckBox;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;


public class DollyCheckBox extends JCheckBox implements ActionListener
{
   private static final String DOLLY = "Dolly";
   private static final long serialVersionUID = -6898471398304979545L;
   private ActiveCameraHolder cameraHolder;

   public DollyCheckBox(ActiveCameraHolder cameraHolder)
   {
      super(DOLLY);
      this.setName(DOLLY);

      this.cameraHolder = cameraHolder;

      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      this.setSelected(camera.isDolly());
      addActionListener(this);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      camera.setDolly(this.isSelected());
   }

   public void makeCameraConsistent()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
            camera.setDolly(isSelected());
         }

      });
   }

   public void makeCheckBoxConsistent()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
            if (camera != null) setSelected(camera.isDolly());
         }

      });
   }

}
