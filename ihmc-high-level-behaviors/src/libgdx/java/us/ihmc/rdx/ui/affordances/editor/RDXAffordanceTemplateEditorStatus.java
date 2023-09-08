package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAffordanceTemplateEditorStatus
{
   private RobotSide activeSide;
   private RDXActiveAffordanceMenu activeMenu;

   public RDXAffordanceTemplateEditorStatus(RobotSide activeSide, RDXActiveAffordanceMenu activeMenu)
   {
      this.activeSide = activeSide;
      this.activeMenu = activeMenu;
   }

   public void setActiveSide(RobotSide side)
   {
      this.activeSide = side;
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
