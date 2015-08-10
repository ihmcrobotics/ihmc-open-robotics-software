package us.ihmc.wanderer.hardware.visualization;

import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.tools.time.TimeTools;
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
   private final JLabel COT_label = new JLabel();
   private final JLabel COT_value = new JLabel();
   private final JLabel avg_vel_label = new JLabel();
   private final JLabel avg_vel_value = new JLabel();
   private final DoubleYoVariable power;
   private final DoubleYoVariable avgpower;
   private final LongYoVariable nanosecondstime;
   private final DoubleYoVariable time;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable COT;
   private final DoubleYoVariable averageVelocity;
   private boolean init_complete = false;
      
   public WandererExpoFrame(YoVariableRegistry parentRegistry, boolean isStandalone)
   {
      super();
      setBounds(0,0,1920,1080);
      final JPanel mainpanel = new JPanel();
      this.add(mainpanel);
      mainpanel.setLayout(new BoxLayout(mainpanel,BoxLayout.Y_AXIS));
      
      final JPanel power_panel = new JPanel();
      power_panel.setLayout(new BoxLayout(power_panel,BoxLayout.X_AXIS));
      power_panel.add(power_label);
      power_label.setText("Motor Power (W): ");
      power_panel.add(Box.createHorizontalGlue());
      power_panel.add(power_value);
      mainpanel.add(power_panel);
      
      final JPanel avgpower_panel = new JPanel();
      avgpower_panel.setLayout(new BoxLayout(avgpower_panel,BoxLayout.X_AXIS));
      avgpower_panel.add(avg_power_label);
      avg_power_label.setText("Avg. Robot Power (W): ");
      avgpower_panel.add(Box.createHorizontalGlue());
      avgpower_panel.add(avg_power_value);
      mainpanel.add(avgpower_panel);
      
      final JPanel time_panel = new JPanel();
      time_panel.setLayout(new BoxLayout(time_panel,BoxLayout.X_AXIS));
      time_panel.add(time_label);
      time_label.setText("Running Time (s): ");
      time_panel.add(Box.createHorizontalGlue());
      time_panel.add(time_value);
      mainpanel.add(time_panel);
      
      final JPanel avg_vel_panel = new JPanel();
      avg_vel_panel.setLayout(new BoxLayout(avg_vel_panel,BoxLayout.X_AXIS));
      avg_vel_panel.add(avg_vel_label);
      avg_vel_label.setText("Speed (m/s): ");
      avg_vel_panel.add(Box.createHorizontalGlue());
      avg_vel_panel.add(avg_vel_value);
      mainpanel.add(avg_vel_panel);
      
      final JPanel COT_panel = new JPanel();
      COT_panel.setLayout(new BoxLayout(COT_panel,BoxLayout.X_AXIS));
      COT_panel.add(COT_label);
      COT_label.setText("COT (W/W): ");
      COT_panel.add(Box.createHorizontalGlue());
      COT_panel.add(COT_value);
      mainpanel.add(COT_panel);
      
      if(isStandalone)
      {
         YoVariableRegistry registry = new YoVariableRegistry("base");
         avgpower = new DoubleYoVariable("averagePower", registry);
         power = new DoubleYoVariable("motorPower", registry);
         nanosecondstime = new LongYoVariable("longtime", registry);
         time = new DoubleYoVariable("time", registry);
         startTime = new DoubleYoVariable("startTime", registry);
         COT = new DoubleYoVariable("COT", registry);
         averageVelocity = new DoubleYoVariable("averageVelocity", registry);
         avgpower.set(456);
         power.set(123);
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
         COT = (DoubleYoVariable) parentRegistry.getVariable("CostOfTransportCalculator","costOfTransport");
         averageVelocity = (DoubleYoVariable) parentRegistry.getVariable("CostOfTransportCalculator","averageVelocity");
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
      COT_value.setFont(power_label.getFont());
      COT_label.setFont(power_label.getFont());
      avg_vel_value.setFont(power_label.getFont());
      avg_vel_label.setFont(power_label.getFont());
      
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
      COT_value.setText(String.format("%.1f",COT.getDoubleValue()));
      avg_vel_value.setText(String.format("%.2f",averageVelocity.getDoubleValue()));
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
