package us.ihmc.wanderer.hardware.visualization;

import java.awt.Component;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class WandererExpoFrame extends JFrame implements PlaybackListener
{
   private static final long serialVersionUID = 9154257744299379146L;
   
   private final JLabel power_label = new JLabel();
   private final JLabel avg_power_label = new JLabel();
   private final JLabel power_value = new JLabel();
   private final JLabel avg_power_value = new JLabel();
   private final JLabel time_label = new JLabel();
   private final JLabel time_value = new JLabel();
   private final DoubleYoVariable power;
   private final DoubleYoVariable avgpower;
   private final LongYoVariable nanosecondstime;
   private final DoubleYoVariable time;
   private final DoubleYoVariable startTime;
   private boolean init_complete = false;
      
   public WandererExpoFrame(YoVariableRegistry parentRegistry, boolean isStandalone)
   {
      super();
      setBounds(0,0,1920,1080);
      final JPanel panel = new JPanel();
      this.add(panel);
      panel.setLayout(new GridLayout(0,2));
      panel.add(power_label);
      power_label.setText("Motor Power: ");
      panel.add(power_value);
      panel.add(avg_power_label);
      avg_power_label.setText("Avg. Power: ");      
      panel.add(avg_power_value);
      panel.add(time_label);
      panel.add(time_value);
      time_label.setText("Running Time: ");      
      
      if(isStandalone)
      {
         YoVariableRegistry registry = new YoVariableRegistry("base");
         avgpower = new DoubleYoVariable("averagePower", registry);
         power = new DoubleYoVariable("motorPower", registry);
         nanosecondstime = new LongYoVariable("longtime", registry);
         time = new DoubleYoVariable("time", registry);
         startTime = new DoubleYoVariable("startTime", registry);
         avgpower.set(123456);
         power.set(123456);
         nanosecondstime.set(1234567890);
         time.set(123456);
         indexChanged(1, 1.0);
      } else
      {
         avgpower = (DoubleYoVariable) parentRegistry.getVariable("powerDistribution","averageRobotPower");
         power = (DoubleYoVariable) parentRegistry.getVariable("Wanderer","totalMotorPower");
         nanosecondstime = (LongYoVariable) parentRegistry.getVariable("SensorProcessing","timestamp");
         time = new DoubleYoVariable("expoTime", parentRegistry);
         startTime = new DoubleYoVariable("expoStartTime", parentRegistry);
      }
      
      if(isStandalone) setupExitOnClose();
      
      setFontSize(100.0f);
   }
   
   private void setFontSize(float size)
   {
      power_label.setFont(power_label.getFont().deriveFont(size));
      avg_power_label.setFont(power_label.getFont());
      time_label.setFont(power_label.getFont());
      power_value.setFont(power_label.getFont());
      avg_power_value.setFont(power_label.getFont());
      time_value.setFont(power_label.getFont());
   }
   
   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      if(!init_complete)
      {
         startTime.set(TimeTools.nanoSecondstoSeconds(nanosecondstime.getLongValue()));
         init_complete = true;
      }
      power_value.setText(String.format("%.1f",power.getDoubleValue()));
      avg_power_value.setText(String.format("%.1f",avgpower.getDoubleValue()));
      time.set(TimeTools.nanoSecondstoSeconds(nanosecondstime.getLongValue())-startTime.getDoubleValue());
      time_value.setText(String.format("%.1f",time.getDoubleValue()));
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
