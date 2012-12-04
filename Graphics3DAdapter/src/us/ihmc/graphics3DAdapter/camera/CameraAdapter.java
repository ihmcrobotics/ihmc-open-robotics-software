package us.ihmc.graphics3DAdapter.camera;

import java.awt.Canvas;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface CameraAdapter
{
   public Canvas getCanvas();
   public void destroy();
}
