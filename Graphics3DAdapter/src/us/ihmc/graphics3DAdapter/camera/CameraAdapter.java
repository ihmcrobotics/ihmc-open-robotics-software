package us.ihmc.graphics3DAdapter.camera;

import javax.swing.JPanel;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface CameraAdapter
{
   public JPanel getPanel();
   public void destroy();
   public void trackNode(Graphics3DNode node);
}
