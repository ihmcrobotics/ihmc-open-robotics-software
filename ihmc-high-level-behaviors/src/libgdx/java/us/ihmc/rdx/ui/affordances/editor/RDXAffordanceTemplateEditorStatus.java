package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAffordanceTemplateEditorStatus
{
   private RobotSide activeSide;
   private RDXActiveAffordanceMenu activeMenu;
   private RDXAffordanceTemplateMirror mirror;

   public RDXAffordanceTemplateEditorStatus(RobotSide activeSide, RDXActiveAffordanceMenu activeMenu)
   {
      this.activeSide = activeSide;
      this.activeMenu = activeMenu;
   }

   public void setMirror(RDXAffordanceTemplateMirror mirror)
   {
      this.mirror = mirror;
   }

   public void setActiveSide(RobotSide side)
   {
      this.activeSide = side;
      if (mirror != null && mirror.isActive())
         mirror.reset();
   }

   public void setActiveMenu(RDXActiveAffordanceMenu menu)
   {
      this.activeMenu = menu;
   }

   public RobotSide getActiveSide()
   {
      return activeSide;
   }

   public RDXActiveAffordanceMenu getActiveMenu()
   {
      return activeMenu;
   }
}
