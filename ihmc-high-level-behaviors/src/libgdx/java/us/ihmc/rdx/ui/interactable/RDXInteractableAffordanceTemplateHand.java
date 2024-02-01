package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.util.ArrayList;
import java.util.List;

public interface RDXInteractableAffordanceTemplateHand extends RenderableProvider
{
   ReferenceFrame getReferenceFrameHand();

   default int getNumberOfFingers()
   {
      if (hasGripper())
         return 2;
      else
         return 0;
   }

   default boolean hasGripper()
   {
      return false;
   }

   default void setGripperClosure(double closure)
   {

   }

   default float getGripperClosure()
   {
      return Float.NaN;
   }

   default float getMinGripperClosure()
   {
      return Float.NaN;
   }

   default float getMaxGripperClosure()
   {
      return Float.NaN;
   }

   default List<String> getAvailableConfigurations()
   {
      return new ArrayList<>();
   }

   default String getConfiguration()
   {
      return null;
   }

   default void setToConfiguration(String configuration)
   {

   }

   default void setToDefaultConfiguration()
   {

   }

   RDXPose3DGizmo getPose3DGizmo();

   boolean isSelected();

   void setSelected(boolean selected);

   default void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   void removeRenderables(RDX3DPanel panel3D);
}