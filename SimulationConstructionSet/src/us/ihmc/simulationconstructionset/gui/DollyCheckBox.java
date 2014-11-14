package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JCheckBox;

import us.ihmc.graphics3DAdapter.camera.CameraPropertiesHolder;


public class DollyCheckBox extends JCheckBox implements ActionListener
{
   private static final long serialVersionUID = -6898471398304979545L;
   private ActiveCameraHolder cameraHolder;

   public DollyCheckBox(ActiveCameraHolder cameraHolder)
   {
      super("Dolly");
      this.setName("Dolly");

      this.cameraHolder = cameraHolder;

      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      this.setSelected(camera.isDolly());
      addActionListener(this);
   }

   public void actionPerformed(ActionEvent actionEvent)
   {
      CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
      camera.setDolly(this.isSelected());
   }

   public void makeCameraConsistent()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
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
         public void run()
         {
            CameraPropertiesHolder camera = cameraHolder.getCameraPropertiesForActiveCamera();
            setSelected(camera.isDolly());
         }

      });
   }

}
