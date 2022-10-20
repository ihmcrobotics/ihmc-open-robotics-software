package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.Color;

public class GDXGizmoTools
{
   public static final Color X_AXIS_DEFAULT_COLOR = new Color(0.9f, 0.4f, 0.4f, 0.4f);
   public static final Color Y_AXIS_DEFAULT_COLOR = new Color(0.4f, 0.9f, 0.4f, 0.4f);
   public static final Color Z_AXIS_DEFAULT_COLOR = new Color(0.4f, 0.4f, 0.9f, 0.4f);
   public static final Color CENTER_DEFAULT_COLOR = new Color(0.7f, 0.7f, 0.7f, 0.4f);

   public static final Color X_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.9f, 0.3f, 0.3f, 0.9f);
   public static final Color Y_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.3f, 0.9f, 0.3f, 0.9f);
   public static final Color Z_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.3f, 0.3f, 0.9f, 0.9f);
   public static final Color CENTER_SELECTED_DEFAULT_COLOR = new Color(0.5f, 0.5f, 0.5f, 0.9f);

   public static final Color[] AXIS_COLORS = {X_AXIS_DEFAULT_COLOR, Y_AXIS_DEFAULT_COLOR, Z_AXIS_DEFAULT_COLOR};
   public static final Color[] AXIS_SELECTED_COLORS = {X_AXIS_SELECTED_DEFAULT_COLOR, Y_AXIS_SELECTED_DEFAULT_COLOR, Z_AXIS_SELECTED_DEFAULT_COLOR};
}
