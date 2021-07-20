package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXTools;

public class FrameGDXNode
{
   private final ReferenceFrame referenceFrame;
   private final ModelInstance modelInstance;

   public FrameGDXNode(ReferenceFrame referenceFrame, ModelInstance modelInstance)
   {
      this.referenceFrame = referenceFrame;
      this.modelInstance = modelInstance;
   }

   public void updatePose()
   {
      GDXTools.toGDX(referenceFrame.getTransformToRoot(), modelInstance.transform);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
