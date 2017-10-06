package us.ihmc.simulationconstructionset.graphics;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;


public class GraphicsDynamicGraphicsObject extends Graphics3DNode implements GraphicsUpdatable
{
   private final YoGraphic yoGraphic;

   public GraphicsDynamicGraphicsObject(YoGraphic yoGraphic)
   {
      super(yoGraphic.getName(), Graphics3DNodeType.VISUALIZATION);
      this.yoGraphic = yoGraphic;
      setGraphicsObject(yoGraphic.getLinkGraphics());
      update();
   }

   @Override
   public void update()
   {
      // IMPORTANT: can't do this here because it causes threading issues. Each thread is responsible for updating its own YoGraphics!
//      yoGraphic.update();
      
      AffineTransform j3dTransform = yoGraphic.getTransform();
      setTransform(j3dTransform);
   }

}
