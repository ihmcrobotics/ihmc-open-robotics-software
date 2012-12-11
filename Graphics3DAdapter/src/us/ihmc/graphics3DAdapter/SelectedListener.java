package us.ihmc.graphics3DAdapter;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface SelectedListener
{
   public void selected(Graphics3DNode graphics3dNode, String modifierKey, Point3d location);
}
