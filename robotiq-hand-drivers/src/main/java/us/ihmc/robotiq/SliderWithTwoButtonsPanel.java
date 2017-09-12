package us.ihmc.robotiq;

import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeListener;

public class SliderWithTwoButtonsPanel extends JPanel
{
   private static final long serialVersionUID = 4136711201068019036L;
   
   private JPanel buttonPanel = new JPanel();
   private JLabel panelLabel;
   private JSlider slider;
   private JButton topButton;
   private JButton bottomButton;
   
   public SliderWithTwoButtonsPanel(String panelLabelText)
   {
      this(panelLabelText, "Send", "Reset");
   }
   
   public SliderWithTwoButtonsPanel(String panelLabelText, String topButtonLabel, String bottomButtonLabel)
   {
      setupPanel(panelLabelText, topButtonLabel, bottomButtonLabel);
   }
   
   private void setupPanel(String panelLabelText, String topButtonLabel, String bottomButtonLabel)
   {
	   panelLabel = new JLabel(panelLabelText);
	   slider = new JSlider(0, 255, 0);
	   topButton = new JButton(topButtonLabel);
	   bottomButton = new JButton(bottomButtonLabel);
	   
	   buttonPanel.add(topButton);
	   buttonPanel.add(bottomButton);
	   
	   this.add(panelLabel);
	   this.add(slider);
	   this.add(buttonPanel);
   }
   
   public void attachSliderActionListener(ChangeListener changeListener)
   {
      slider.addChangeListener(changeListener);
   }
   
   public void attachTopButtonActionListener(ActionListener actionListener)
   {
      topButton.addActionListener(actionListener);
   }
   
   public void attachBottomButtonActionListener(ActionListener actionListener)
   {
      bottomButton.addActionListener(actionListener);
   }
   
   public JSlider getSlider()
   {
	   return slider;
   }
}
