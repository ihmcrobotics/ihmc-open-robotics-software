package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.Color;

public class RDXGizmoTools
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

   public static final Color DISABLED_AXIS_COLOR = new Color(0.5f, 0.5f, 0.5f, 0.2f);
   /** When wanting to go faster or move more and the fine amount isn't enough, go this times as much. */
   public static final double FINE_TO_COARSE_MULTIPLIER = 10.0;
   /** Maybe you want to nudge things by this amount. */
   public static final double INITIAL_SMALL_STEP = 0.01;
   /** A coarse nudge amount. */
   public static final double INITIAL_BIG_STEP = INITIAL_SMALL_STEP * FINE_TO_COARSE_MULTIPLIER;
   /** Maybe you want to nudge rotations by this amount. */
   public static final double INITIAL_FINE_ROTATION = 0.5;
   /** A coarse nudge amount. */
   public static final double INITIAL_COARSE_ROTATION = INITIAL_FINE_ROTATION * FINE_TO_COARSE_MULTIPLIER;

   /** How much the zoom has to change to rebuild the gizmo mesh, if doing that. A millimeter is plenty. */
   public static final double ZOOM_RESIZE_EPSILON = 1e-3;
}
