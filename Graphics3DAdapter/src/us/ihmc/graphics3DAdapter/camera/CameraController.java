package us.ihmc.graphics3DAdapter.camera;


import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.input.KeyListener;
import us.ihmc.graphics3DAdapter.input.MouseListener;
import us.ihmc.graphics3DAdapter.input.SelectedListener;


public interface CameraController extends KeyListener, MouseListener, SelectedListener
{
   public void computeTransform(Transform3D cameraTransform);
}
