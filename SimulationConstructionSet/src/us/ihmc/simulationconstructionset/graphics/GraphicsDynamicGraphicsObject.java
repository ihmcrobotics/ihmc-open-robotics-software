package us.ihmc.simulationconstructionset.graphics;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.GraphicsUpdatable;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.graphics.YoGraphic;


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

   public void update()
   {
      // IMPORTANT: can't do this here because it causes threading issues. Each thread is responsible for updating its own DynamicGraphicObjects!
//      yoGraphic.update();
      
      RigidBodyTransform j3dTransform = yoGraphic.getTransform();
      setTransform(j3dTransform);
   }


}
