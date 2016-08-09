package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.plotting.Artifact;
import us.ihmc.robotics.geometry.Transform3d;

public class NonRewindableYoGraphic extends YoGraphic
{
   private Graphics3DObject graphics3DObject;

   public NonRewindableYoGraphic(String name, Graphics3DObject graphics3DObject)
   {
      super(name);

      this.graphics3DObject = graphics3DObject;
   }

   @Override
   protected void computeRotationTranslation(Transform3d transform3D)
   {

   }

   @Override
   protected boolean containsNaN()
   {
      return false;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3DObject;
   }

   @Override
   public Artifact createArtifact()
   {
      return null;
   }
}
