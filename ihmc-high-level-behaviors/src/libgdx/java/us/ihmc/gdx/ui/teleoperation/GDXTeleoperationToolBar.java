package us.ihmc.gdx.ui.teleoperation;

import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.ui.GDX3DPanelToolbarButton;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;

// TODO: Extract tool bar method from GDX3DPanel
public class GDXTeleoperationToolBar
{
   // NOTE: FOR ICONS (NON-BUTTON)
   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private final float iconSize = 35.0f;
   // NOTE: ICON TEXTURES
   private GDXIconTexture homePoseIcon;
   private GDXIconTexture standPrepIcon;
   private GDXIconTexture abortIcon;
   private GDXIconTexture pauseIcon;
   private GDXIconTexture continueIcon;
   private GDXIconTexture shutdownIcon;
   private final boolean hotButtonTesting = true;
   private ArrayList<GDX3DPanelToolbarButton> buttons = new ArrayList<>();
   private ArrayList<GDXIconTexture> iconTextures;
   // FIXME: Need proper (generalized) way to implement toggling later.
   private int stateIndex = 0;

   // TODO: Implement methods here
}
