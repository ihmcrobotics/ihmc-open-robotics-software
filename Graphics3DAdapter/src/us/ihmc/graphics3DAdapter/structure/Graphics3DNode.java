package us.ihmc.graphics3DAdapter.structure;

import java.util.ArrayList;

import com.yobotics.simulationconstructionset.LinkGraphics;

import us.ihmc.graphics3DAdapter.generics.GenericTransform;

public interface Graphics3DNode
{
   public void packTransformToParent(GenericTransform transform);
   public ArrayList<Graphics3DNode> getChildrenNodes();
   
   //TODO: Move LinkGraphics to graphics3DAdapter and rename to GraphicObject
   public LinkGraphics getGraphicObject();
}
