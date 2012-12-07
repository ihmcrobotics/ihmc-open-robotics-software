package us.ihmc.graphics3DAdapter.camera;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface CameraController
{
   public void useFreeFlyingCamera();
   public void trackNode(Graphics3DNode node);
   public void attachCameraToNode(Graphics3DNode node, Transform3D offset);
}
