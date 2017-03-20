package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * This interface is applied to YoGraphics and Artifacts which support
 * being packed into YoVariables and sent from simulation to viewer.
 *
 * See YoGraphicFactory and YoVariableHandshakeServer
 *
 * @author Alex Lesman
 *
 */
public interface RemoteYoGraphic
{
   /*
    * WARNING: Changing the order here will break old logs. Only add new elements to the end of the list
    */
   public enum RemoteGraphicType
   {
      CYLINDER_DGO,
      COORDINATE_SYSTEM_DGO,
      TRIANGLE_DGO,
      POSITION_DGO,
      EMPTY_SLOT_2_DGO,
      EMPTY_SLOT_3_DGO,
      VECTOR_DGO,
      YO_FRAME_POLYGON_DGO,
      LINE_SEGMENT_DGO,
      EMPTY_SLOT_4_DGO,
      POSITION_ARTIFACT,
      CIRCLE_ARTIFACT,
      LINE_SEGMENT_2D_ARTIFACT,
      POLYGON_ARTIFACT,
      LINE_ARTIFACT,
      PLANAR_REGIONS_LIST_DGO
   }

   public String getName();

   public RemoteGraphicType getRemoteGraphicType();

   public YoVariable<?>[] getVariables();

   public double[] getConstants();

   public AppearanceDefinition getAppearance();
}
