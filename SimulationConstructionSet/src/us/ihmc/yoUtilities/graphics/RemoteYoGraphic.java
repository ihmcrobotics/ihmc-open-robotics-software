package us.ihmc.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * This interface is applied to DynamicGraphicObjects and Artifacts which support
 * being packed into YoVariables and sent from simulation to viewer.
 *
 * See DynamicGraphicFactory and YoVariableHandshakeServer
 * 
 * @author Alex Lesman
 *
 */
public interface RemoteYoGraphic
{
   public enum RemoteGraphicType
   {
      CYLINDER_DGO,
      COORDINATE_SYSTEM_DGO,
      EMPTY_SLOT_1_DGO,
      POSITION_DGO,
      EMPTY_SLOT_2_DGO, 
      EMPTY_SLOT_3_DGO,
      VECTOR_DGO,
      YO_FRAME_POLYGON_DGO,
      LINE_SEGMENT_DGO,
      EMPTY_SLOT_4_DGO,
      POSITION_ARTIFACT,
      LINE_SEGMENT_2D_ARTIFACT,
      POLYGON_ARTIFACT
   }

   public String getName();

   public RemoteGraphicType getRemoteGraphicType();

   public YoVariable<?>[] getVariables();

   public double[] getConstants();

   public AppearanceDefinition getAppearance();
}
