package us.ihmc.robotiq;

import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeListener;

public class SliderWithTwoButtonsPanel extends JPanel
{
   private static final long serialVersionUID = 4136711201068019036L;
   
   private JSlider slider;
   
   private JPanel buttonPanel = new JPanel();
   private JButton topButton;
   private JButton bottomButton;
   
   public SliderWithTwoButtonsPanel()
   {
      this("Send", "Reset");
   }
   
   public SliderWithTwoButtonsPanel(String topButtonLabel, String bottomButtonLabel)
   {
      setupPanel(topButtonLabel, bottomButtonLabel);
   }
   
   private void setupPanel(String topButtonLabel, String bottomButtonLabel)
   {
      slider = new JSlider();
      
      topButton = new JButton(topButtonLabel);
      bottomButton = new JButton(bottomButtonLabel);
      
      buttonPanel.add(topButton);
      buttonPanel.add(bottomButton);
      
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
}
