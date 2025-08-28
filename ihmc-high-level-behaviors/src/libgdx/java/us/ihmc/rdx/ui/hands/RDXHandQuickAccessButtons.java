package us.ihmc.rdx.ui.hands;

import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.hands.RDXHandInterface.HandAction;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.rdx.ui.hands.RDXHandInterface.HandAction.*;

public class RDXHandQuickAccessButtons
{
   private final RDX3DPanelToolbarButton openHandCalibrateButton;
   private final RDX3DPanelToolbarButton closeHandResetButton;
   private final RDXIconTexture openIcon;
   private final RDXIconTexture calibrateIcon;
   private final RDXIconTexture closeIcon;
   private final RDXIconTexture resetIcon;
   private final String openHandText;
   private final String calibrateHandText;
   private final String closeHandText;
   private final String resetHandText;

   private final RDXHandInterface hand;

   public RDXHandQuickAccessButtons(RDXBaseUI baseUI, RDXHandInterface hand)
   {
      this.hand = hand;

      RobotSide side = hand.getSide();
      if (side == RobotSide.LEFT) // Make buttons symmetrical
      {
         closeHandResetButton = baseUI.getPrimary3DPanel().addToolbarButton();
         openHandCalibrateButton = baseUI.getPrimary3DPanel().addToolbarButton();
      }
      else
      {
         openHandCalibrateButton = baseUI.getPrimary3DPanel().addToolbarButton();
         closeHandResetButton = baseUI.getPrimary3DPanel().addToolbarButton();
      }

      calibrateIcon = openHandCalibrateButton.loadAndSetIcon("icons/calibrate.png");
      openIcon = openHandCalibrateButton.loadAndSetIcon("icons/openHand%s.png".formatted(side.getPascalCaseName()));
      openHandText = "Open %s hand".formatted(side.getLowerCaseName());
      calibrateHandText = "Calibrate %s hand".formatted(side.getLowerCaseName());
      openHandCalibrateButton.setTooltipText(openHandText);
      openHandCalibrateButton.setOnPressed(() -> hand.sendCommand(OPEN));

      resetIcon = closeHandResetButton.loadAndSetIcon("icons/resetHand.png");
      closeIcon = closeHandResetButton.loadAndSetIcon("icons/closeHand%s.png".formatted(side.getPascalCaseName()));
      closeHandText = "Close %s hand".formatted(side.getLowerCaseName());
      resetHandText = "Reset %s hand".formatted(side.getLowerCaseName());
      closeHandResetButton.setTooltipText(closeHandText);
      closeHandResetButton.setOnPressed(() -> hand.sendCommand(CLOSE));
   }

   public void update()
   {
      boolean isCalibrated = hand.isCalibrated();
      boolean needsReset = hand.needsReset();

      openHandCalibrateButton.setIconTexture(isCalibrated ? openIcon : calibrateIcon);
      openHandCalibrateButton.setOnPressed(needsReset ? null : () -> hand.sendCommand(isCalibrated ? OPEN : CALIBRATE));
      openHandCalibrateButton.setTooltipText(needsReset ? "Unavailable" : isCalibrated ? openHandText : calibrateHandText);
      closeHandResetButton.setIconTexture(needsReset ? resetIcon : closeIcon);
      closeHandResetButton.setOnPressed(() -> hand.sendCommand(needsReset ? RESET : CLOSE));
      closeHandResetButton.setTooltipText(needsReset ? resetHandText : closeHandText);
   }
}
