package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.rdx.ui.interactable.RDXInteractableAffordanceTemplateHand;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAffordanceTemplateEditorStatus
{
   private RobotSide activeSide;
   private RDXActiveAffordanceMenu activeMenu;
   private Class<? extends RDXInteractableAffordanceTemplateHand> handModel;

   public RDXAffordanceTemplateEditorStatus(RobotSide activeSide, RDXActiveAffordanceMenu activeMenu, Class<? extends RDXInteractableAffordanceTemplateHand> handModel)
   {
      this.activeSide = activeSide;
      this.activeMenu = activeMenu;
      this.handModel = handModel;
   }

   public void setActiveSide(RobotSide side)
   {
      this.activeSide = side;
   }

   public void setActiveMenu(RDXActiveAffordanceMenu menu)
   {
      this.activeMenu = menu;
   }

   public void setActiveHandModel(Class<? extends RDXInteractableAffordanceTemplateHand> handModel)
   {
      this.handModel = handModel;
   }

   public RobotSide getActiveSide()
   {
      return activeSide;
   }

   public RDXActiveAffordanceMenu getActiveMenu()
   {
      return activeMenu;
   }

   public Class<?> getActiveHandModel()
   {
      return handModel;
   }
}
