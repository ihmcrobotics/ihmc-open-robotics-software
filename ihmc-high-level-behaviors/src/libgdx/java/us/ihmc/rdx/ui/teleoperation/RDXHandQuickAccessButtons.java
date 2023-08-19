package us.ihmc.rdx.ui.teleoperation;

import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXHandQuickAccessButtons
{
   private final RDX3DPanelToolbarButton openHandCalibrateButton;
   private final Runnable openHand;
   private final Runnable closeHand;
   private final Runnable calibrateHand;
   private final Runnable resetHand;
   private final RDX3DPanelToolbarButton closeHandResetButton;
   private final RDXIconTexture openIcon;
   private final RDXIconTexture calibrateIcon;
   private final RDXIconTexture closeIcon;
   private final RDXIconTexture resetIcon;

   public RDXHandQuickAccessButtons(RDXBaseUI baseUI,
                                    RobotSide side,
                                    Runnable openHand,
                                    Runnable closeHand,
                                    Runnable calibrateHand,
                                    Runnable resetHand)
   {
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

      this.openHand = openHand;
      this.closeHand = closeHand;
      this.calibrateHand = calibrateHand;
      this.resetHand = resetHand;

      calibrateIcon = openHandCalibrateButton.loadAndSetIcon("icons/calibrate.png");
      openIcon = openHandCalibrateButton.loadAndSetIcon("icons/openHand%s.png".formatted(side.getPascalCaseName()));
      openHandCalibrateButton.setTooltipText("Open %s hand".formatted(side.getLowerCaseName()));
      openHandCalibrateButton.setOnPressed(openHand);

      resetIcon = closeHandResetButton.loadAndSetIcon("icons/resetHand.png");
      closeIcon = closeHandResetButton.loadAndSetIcon("icons/closeHand%s.png".formatted(side.getPascalCaseName()));
      closeHandResetButton.setTooltipText("Close %s hand".formatted(side.getLowerCaseName()));
      closeHandResetButton.setOnPressed(closeHand);
   }

   public void update(RDXSakeHandInformation sakeHandInfo)
   {
      openHandCalibrateButton.setIconTexture(sakeHandInfo.getCalibrated() ? openIcon : calibrateIcon);
      openHandCalibrateButton.setOnPressed(sakeHandInfo.getCalibrated() ? openHand : calibrateHand);
      closeHandResetButton.setIconTexture(sakeHandInfo.getNeedsReset() ? resetIcon : closeIcon);
      closeHandResetButton.setOnPressed(sakeHandInfo.getNeedsReset() ? resetHand : closeHand);
   }
}
