package us.ihmc.wanderer.hardware.visualization;

import java.awt.GridLayout;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class WandererExpoFrame extends JFrame implements PlaybackListener
{
   private static final long serialVersionUID = 9154257744299379146L;
   
   private final JLabel power_label = new JLabel();
   private final JLabel time_label = new JLabel();
   private final DoubleYoVariable power;
   private final LongYoVariable time;
      
   public WandererExpoFrame(YoVariableHolder yoVariableHolder, boolean isStandalone)
   {
      super();
      setBounds(0,0,1920,1080);
      final JPanel panel = new JPanel();
      final JLabel label = new JLabel();
      final JLabel label2 = new JLabel();
      this.add(panel);
      panel.setLayout(new GridLayout(0,2));
      panel.add(label);
      panel.add(power_label);
      panel.add(label2);
      panel.add(time_label);
      label.setText("Robot Power: ");
      label2.setText("Running Time: ");
      
      if(isStandalone)
      {
         power = null;
         time = null;
      } else
      {
         power = (DoubleYoVariable) yoVariableHolder.getVariable("Wanderer","totalMotorPower");
         time = (LongYoVariable) yoVariableHolder.getVariable("SensorProcessing","timestamp");
      }
      
      if(isStandalone) setupExitOnClose();
      
      label.setFont(label.getFont().deriveFont(100.0f));
      label2.setFont(label.getFont().deriveFont(100.0f));
      power_label.setFont(label.getFont().deriveFont(100.0f));
      time_label.setFont(label.getFont().deriveFont(100.0f));
   }
   
   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      power_label.setText(String.format("%.1f",power.getDoubleValue()));
      time_label.setText(String.format("%.1f",TimeTools.nanoSecondstoSeconds(time.getLongValue())));
   }

   @Override
   public void play(double realTimeRate)
   {  
   }

   @Override
   public void stop()
   {      
   }
   
   
   public static void main(String[] args)
   {
      WandererExpoFrame wef = new WandererExpoFrame(null,true);
      wef.setVisible(true);
   }
   
   private void setupExitOnClose()
   {
      this.addComponentListener(new ComponentListener()
      {         
         @Override
         public void componentShown(ComponentEvent e) {}
         
         @Override
         public void componentResized(ComponentEvent e){}
         
         @Override
         public void componentMoved(ComponentEvent e) {}
         
         @Override
         public void componentHidden(ComponentEvent e)
         {
            System.exit(0);            
         }
      });
   }
   
}
