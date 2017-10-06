package us.ihmc.wanderer.hardware.visualization;

import java.awt.Color;
import java.awt.Font;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.Conversions;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.simulationconstructionset.PlaybackListener;

public class WandererExpoFrame extends JFrame implements PlaybackListener
{
   private static final long serialVersionUID = 9154257744299379146L;
   
   private final JLabel title_label = new JLabel("Performance Since Last Charge");
   private final JLabel walking_time_label = new JLabel("Walking Time: ");
   private final JLabel operating_time_label = new JLabel("Operating Time: ");
   private final JLabel stepCount_label = new JLabel("Step Count: ");
   private final JLabel distance_label = new JLabel("Distance Traveled (m): ");
   private final JLabel battery_label = new JLabel("Battery Level: ");
   private final JLabel walking_time_value = new JLabel();
   private final JLabel operating_time_value = new JLabel();
   private final JLabel stepCount_value = new JLabel();
   private final JLabel distance_value = new JLabel();
   private final JLabel battery_value = new JLabel();
   
   private final JLabel subsection_label = new JLabel("Current Performance");
   private final JLabel avg_power_label = new JLabel("Current Power Consumption (W): ");
   private final JLabel avg_vel_label = new JLabel("Current Walking Speed (m/s): ");
   private final JLabel COT_label = new JLabel("Cost of Transport: ");
   private final JLabel avg_power_value = new JLabel();
   private final JLabel avg_vel_value = new JLabel();
   private final JLabel COT_value = new JLabel();   

   private double initTime;
   private double lastStartWalkTime;
   private double internalPriorWalkingDuration;
   private int stepCount = 0;
   private final YoLong nanosecondstime;
   private final YoDouble total_time;
   private final YoDouble startTime;
   private final YoDouble walking_time;
   private final YoBoolean expo_isWalking;
   private final YoDouble priorWalkingDuration;
   private final YoLong startingStepCount;
   //private final YoEnum<WalkingState> walkingState;
   private final YoDouble distanceTraveled;
   private final YoDouble startDistance;
   private final YoDouble batteryLevel;
   //private final AlphaFilteredYoVariable batteryLevel_filt;
   
   private final YoDouble avgpower;
   private final YoDouble averageVelocity;
   private final YoDouble COT;
   
   private final Font titleFont = new Font("Calibri",Font.ITALIC, 75);
   private final Font mainLabelFont = new Font("Calibri",Font.PLAIN, 100);
   private final Font mainValueFont = new Font("Calibri",Font.BOLD, 100);
   private final Font subsectionFont = new Font("Calibri",Font.ITALIC, 45);
   private final Font subLabelFont = new Font("Calibri",Font.PLAIN, 40);
   private final Font subValueFont = new Font("Calibri",Font.BOLD, 40);
   
   private boolean init_complete = false;
      
   public WandererExpoFrame(YoVariableRegistry parentRegistry, boolean isStandalone)
   {
      super();
      setBounds(0,0,1920,1080);
      final JPanel mainpanel = new JPanel();
      this.add(mainpanel);
      mainpanel.setLayout(new BoxLayout(mainpanel,BoxLayout.Y_AXIS));
      mainpanel.setAlignmentX(CENTER_ALIGNMENT);
      
      createTitleLabels(mainpanel);
      createOperatingTimeLabels(mainpanel);
      createWalkingTimeLabels(mainpanel);
      createStepCountLabels(mainpanel);
      createDistanceLabels(mainpanel);
      createBatteryLabels(mainpanel);
      
      createSubsectionLabels(mainpanel);
      createAveragePowerLabels(mainpanel);
      createAverageVelocityLabels(mainpanel);
      createCOTLabels(mainpanel);
      //createMotorPowerLabels(mainpanel);
      
      if(isStandalone)
      {
         YoVariableRegistry registry = new YoVariableRegistry("base");
         nanosecondstime = new YoLong("longtime", registry);
         total_time = new YoDouble("time", registry);
         startTime = new YoDouble("startTime", registry);
         walking_time = new YoDouble("walkingtime", registry);
         expo_isWalking = new YoBoolean("walk", registry);
         startingStepCount = new YoLong("expoStartingStepCount", registry);
         //walkingState = new YoEnum<WalkingState>("walkingState", registry, WalkingState.class);
         priorWalkingDuration = new YoDouble("startWalkingTime", registry);
         distanceTraveled = new YoDouble("distanceTraveled", registry);
         startDistance = new YoDouble("startDistance", registry);
         batteryLevel = new YoDouble("batteryLevel", registry);
         avgpower = new YoDouble("averagePower", registry);
         averageVelocity = new YoDouble("averageVelocity", registry);
         COT = new YoDouble("COT", registry);
         
         nanosecondstime.set(1234567890);
         startTime.set(0.0);
         expo_isWalking.set(false);
         priorWalkingDuration.set(0.0);
         walking_time.set(6100.0);
         init_complete = true;
         startingStepCount.set(123);
         distanceTraveled.set(1.0);
         startDistance.set(0.0);
         batteryLevel.set(86);
         avgpower.set(456);
         notifyOfIndexChange(1);
      } else
      {
         nanosecondstime = (YoLong) parentRegistry.getVariable("SensorProcessing","timestamp");
         total_time = new YoDouble("expoTime", parentRegistry);
         startTime = new YoDouble("expoStartTime", parentRegistry);
         walking_time = new YoDouble("expoWalkingTime", parentRegistry);
         expo_isWalking = (YoBoolean) parentRegistry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper","walk");
         startingStepCount = new YoLong("expoStartingStepCount", parentRegistry);
         //walkingState = (YoEnum<WalkingState>) parentRegistry.getVariable("WalkingHighLevelHumanoidController", "walkingState");
         priorWalkingDuration = new YoDouble("expoStartWalkingTime", parentRegistry);
         distanceTraveled = (YoDouble) parentRegistry.getVariable("CostOfTransportCalculator","distanceTraveled");
         startDistance = new YoDouble("expoStartDistance", parentRegistry);
         batteryLevel = (YoDouble) parentRegistry.getVariable("WandererBatteryMonitor","totalBatteryVoltage");
         
         avgpower = (YoDouble) parentRegistry.getVariable("powerDistribution","averageRobotPower");
         averageVelocity = (YoDouble) parentRegistry.getVariable("CostOfTransportCalculator","averageVelocity");
         COT = (YoDouble) parentRegistry.getVariable("CostOfTransportCalculator","costOfTransport");
         //power = (YoDouble) parentRegistry.getVariable("Wanderer","totalMotorPower");
      }
      
      if(isStandalone) setupExitOnClose();
      
      expo_isWalking.addVariableChangedListener(new VariableChangedListener()
      {         
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            manageWalkChanged(v.getValueAsDouble()>0.5);
         }
      });
    /*  walkingState.addVariableChangedListener(new VariableChangedListener()
      {         
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            manageWalkingStateChanged((YoEnum<WalkingState>) v);
         }
      }); */
   }   
      
   private void createTitleLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      parentPanel.add(panel);
      panel.add(title_label);
      title_label.setFont(titleFont);
   }
   
   private void createWalkingTimeLabels(JPanel parentPanel)
   {
      final JPanel time_panel = new JPanel();
      time_panel.setLayout(new BoxLayout(time_panel,BoxLayout.X_AXIS));
      time_panel.add(walking_time_label);
      time_panel.add(Box.createHorizontalGlue());
      time_panel.add(walking_time_value);
      parentPanel.add(time_panel);
      walking_time_label.setFont(mainLabelFont);
      walking_time_value.setFont(mainValueFont);
      walking_time_value.setForeground(Color.RED);
   }
     
   private void createOperatingTimeLabels(JPanel parentPanel)
   {
      final JPanel time_panel = new JPanel();
      time_panel.setLayout(new BoxLayout(time_panel,BoxLayout.X_AXIS));
      time_panel.add(operating_time_label);
      time_panel.add(Box.createHorizontalGlue());
      time_panel.add(operating_time_value);
      parentPanel.add(time_panel);
      operating_time_label.setFont(mainLabelFont);
      operating_time_value.setFont(mainValueFont);
      operating_time_value.setForeground(Color.RED);
   }
   
   private void createStepCountLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(stepCount_label);
      panel.add(Box.createHorizontalGlue());
      panel.add(stepCount_value);
      parentPanel.add(panel);
      stepCount_label.setFont(mainLabelFont);
      stepCount_value.setFont(mainValueFont);
      stepCount_value.setForeground(Color.RED);
   }
   
   private void createDistanceLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(distance_label);
      panel.add(Box.createHorizontalGlue());
      panel.add(distance_value);
      parentPanel.add(panel);
      distance_label.setFont(mainLabelFont);
      distance_value.setFont(mainValueFont);
      distance_value.setForeground(Color.RED);
   }
   
   private void createBatteryLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(battery_label);
      panel.add(Box.createHorizontalGlue());
      panel.add(battery_value);
      parentPanel.add(panel);
      battery_label.setFont(mainLabelFont);
      battery_value.setFont(mainValueFont);
      battery_value.setForeground(Color.RED);
   }
   
   private void createSubsectionLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(subsection_label);
      panel.add(Box.createHorizontalGlue());
      parentPanel.add(panel);
      subsection_label.setFont(subsectionFont);
   }
   
   private void createAveragePowerLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(avg_power_label);
      panel.add(avg_power_value);
      panel.add(Box.createHorizontalGlue());
      parentPanel.add(panel);
      avg_power_label.setFont(subLabelFont);
      avg_power_value.setFont(subValueFont);
   }
   
   private void createAverageVelocityLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(avg_vel_label);
      panel.add(avg_vel_value);
      panel.add(Box.createHorizontalGlue());
      parentPanel.add(panel);
      avg_vel_label.setFont(subLabelFont);
      avg_vel_value.setFont(subValueFont);
   }
      
   private void createCOTLabels(JPanel parentPanel)
   {
      final JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel,BoxLayout.X_AXIS));
      panel.add(COT_label);
      panel.add(COT_value);
      panel.add(Box.createHorizontalGlue());
      parentPanel.add(panel);
      COT_label.setFont(subLabelFont);
      COT_value.setFont(subValueFont);
   }
   
   private void manageWalkChanged(boolean nowWalking)
   {
      if(nowWalking)
         lastStartWalkTime = Conversions.nanosecondsToSeconds(nanosecondstime.getLongValue());
      else
         internalPriorWalkingDuration = walking_time.getDoubleValue();      
   }
   
   private void manageWalkingStateChanged(YoEnum<WalkingStateEnum> walkingState)
   {
      switch(walkingState.getEnumValue())
      {
         case TO_WALKING_LEFT_SUPPORT:
         case TO_WALKING_RIGHT_SUPPORT:
         case TO_STANDING:
            stepCount++;
            break;
         default:
            break;
      }
   }
   
   @Override
   public void notifyOfIndexChange(int newIndex)
   {
      if(!init_complete)
      {
         initTime = Conversions.nanosecondsToSeconds(nanosecondstime.getLongValue());
         init_complete = true;
      }
      if(expo_isWalking.getBooleanValue())
         walking_time.set(Conversions.nanosecondsToSeconds(nanosecondstime.getLongValue())-lastStartWalkTime+internalPriorWalkingDuration);
      total_time.set(Conversions.nanosecondsToSeconds(nanosecondstime.getLongValue())-initTime+startTime.getDoubleValue());

      walking_time_value.setText(displayTime((int)walking_time.getDoubleValue()+(int)priorWalkingDuration.getDoubleValue()));
      operating_time_value.setText(displayTime((int)total_time.getDoubleValue()));
      stepCount_value.setText(String.format("%d",(int)(walking_time.getDoubleValue()*48.0/60.0)+startingStepCount.getLongValue()));
      distance_value.setText(String.format("%.1f",distanceTraveled.getDoubleValue()+startDistance.getDoubleValue()));
      battery_value.setText(String.format("%.1f%%",batteryPercentage(batteryLevel.getDoubleValue())));
      
      avg_power_value.setText(String.format("%.1f",avgpower.getDoubleValue()));
      avg_vel_value.setText(String.format("%.2f",averageVelocity.getDoubleValue()));      
      COT_value.setText(String.format("%.1f",COT.getDoubleValue()));
   }

   private String displayTime(int time)
   {
      int hours = time/3600;
      int minutes = (time % 3600)/60;
      int seconds = (time % 60);
      if(minutes>9)
         if(seconds>9)
            return String.format("%d:%d:%d",hours,minutes,seconds);
         else
            return String.format("%d:%d:0%d",hours,minutes,seconds);
      else
         if(seconds>9)
            return String.format("%d:0%d:%d",hours,minutes,seconds);
         else
            return String.format("%d:0%d:0%d",hours,minutes,seconds);
   }
   
   private double batteryPercentage(double voltage)
   {
      return 100.0*(voltage-84)/(107-84);
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
