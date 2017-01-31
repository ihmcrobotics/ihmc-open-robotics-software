package us.ihmc.wanderer.hardware;

import java.awt.Color;
import java.awt.Component;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.EnumMap;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableCellRenderer;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wanderer.hardware.visualization.WandererExpoFrame;

public class WandererDashboard extends JPanel implements PlaybackListener
{
   private static final long serialVersionUID = -5987765792397743483L;

   private JTable table;

   private final EnumMap<WandererActuator, YoVariable<?>> motorTemperatures = new EnumMap<>(WandererActuator.class);
   private final EnumMap<WandererActuator, YoVariable<?>> mcbTemperatures1 = new EnumMap<>(WandererActuator.class);
   private final EnumMap<WandererActuator, YoVariable<?>> mcbTemperatures2 = new EnumMap<>(WandererActuator.class);
   private final EnumMap<WandererActuator, YoVariable<?>> consecutivePacketDropCount = new EnumMap<>(WandererActuator.class);
   private final EnumMap<WandererActuator, YoVariable<?>> motorEncoders = new EnumMap<>(WandererActuator.class);
   

   public static void createDashboard(final SimulationConstructionSet scs, YoVariableRegistry registry)
   {
      final WandererDashboard wandererDashboard = new WandererDashboard(registry);
      scs.addExtraJpanel(wandererDashboard, "Dashboard", true);
      scs.attachPlaybackListener(wandererDashboard);
      
      final WandererExpoFrame wandererExpoFrame = new WandererExpoFrame(registry,false);
      wandererExpoFrame.setVisible(true);
      scs.attachPlaybackListener(wandererExpoFrame);
      
      JButton showDashboard = new JButton("Show dashboard");
      showDashboard.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            scs.getStandardSimulationGUI().selectPanel("Dashboard");
         }
      });
      
      scs.addButton(showDashboard);
   }
   
   private WandererDashboard(YoVariableHolder yoVariableHolder)
   {
      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

      //createPowerButtons(yoVariableHolder);
      //createCalibrationButtons(yoVariableHolder);
      createTable(yoVariableHolder);
      createInitializationButtons(yoVariableHolder);

   }

   private void createCalibrationButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel calibrationPanel = new JPanel();
      calibrationPanel.setLayout(new BoxLayout(calibrationPanel, BoxLayout.X_AXIS));
      final YoVariable<?> updateOffsets = yoVariableHolder.getVariable("Wanderer", "updateOffsets");
      final YoVariable<?> tareSensorsVar = yoVariableHolder.getVariable("Wanderer", "tareSensors");
      JButton setOffsets = new JButton("Update offsets");
      setOffsets.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            updateOffsets.setValueFromDouble(1.0);
         }
      });
      calibrationPanel.add(setOffsets);
      JButton tareSensors = new JButton("Tare sensors");
      tareSensors.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            tareSensorsVar.setValueFromDouble(1.0);
         }
      });
      calibrationPanel.add(tareSensors);
      add(calibrationPanel);
   }
   
   
   private void createInitializationButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel initializationPanel = new JPanel();
      initializationPanel.setLayout(new BoxLayout(initializationPanel, BoxLayout.X_AXIS));
      final BooleanYoVariable enabledOutput = (BooleanYoVariable)yoVariableHolder.getVariable("WandererOutputWriter","enableOutput");
      final BooleanYoVariable startStandPrep = (BooleanYoVariable)yoVariableHolder.getVariable("WandererStandPrep","startStandPrep");
      final EnumYoVariable requestedHighLevelState = (EnumYoVariable)yoVariableHolder.getVariable("HighLevelHumanoidControllerManager","requestedHighLevelState");
      final DoubleYoVariable controlRatio = (DoubleYoVariable) yoVariableHolder.getVariable("WandererOutputWriter","controlRatio");
      
      final DoubleYoVariable leftFootForce = (DoubleYoVariable)yoVariableHolder.getVariable("l_footStateEstimatorWrenchBasedFootSwitch","l_footStateEstimatorFootForceMag");
      final DoubleYoVariable rightFootForce = (DoubleYoVariable)yoVariableHolder.getVariable("r_footStateEstimatorWrenchBasedFootSwitch","r_footStateEstimatorFootForceMag");
            
      final JButton enabledOutputButton = new JButton("Enable torque output");
      enabledOutputButton.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
        	 if(!enabledOutput.getBooleanValue())
        	 {
        		 enabledOutput.set(true);
        		 enabledOutputButton.setText("Disable torque output");
        	 } else {
        		 enabledOutput.set(false);
        		 enabledOutputButton.setText("Enable torque output");        		 
        	 }
         }
      });
      initializationPanel.add(enabledOutputButton);

      JButton startStandPrepButton = new JButton("Start StandPrep");
      startStandPrepButton.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            startStandPrep.set(true);;
         }
      });
      initializationPanel.add(startStandPrepButton);
      
      final JButton switchToWalk = new JButton("Switch to Walk");
      switchToWalk.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            requestedHighLevelState.set(HighLevelState.WALKING.ordinal());
            ThreadTools.sleep(500);
            requestedHighLevelState.set(HighLevelState.WALKING.ordinal());
            ThreadTools.sleep(500);
            controlRatio.set(1.0);
         }
      });
      initializationPanel.add(switchToWalk);
      
      VariableChangedListener robotOnGroundChecker = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if((leftFootForce.getDoubleValue() + rightFootForce.getDoubleValue()) > 400.0)
            {	
            	switchToWalk.setEnabled(true);
            	switchToWalk.setBackground(Color.GREEN);
            }
            else
            {
            	switchToWalk.setEnabled(false);
                switchToWalk.setBackground(Color.RED);
            }
         }
      };
      try
      {
         //TODO: Fix Me!
      	leftFootForce.addVariableChangedListener(robotOnGroundChecker);
      	rightFootForce.addVariableChangedListener(robotOnGroundChecker);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      add(initializationPanel);
      
      
   }
   

   private void createPowerButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel powerPanel = new JPanel();
      powerPanel.setLayout(new BoxLayout(powerPanel, BoxLayout.X_AXIS));
      final YoVariable<?> logicPowerStateRequest = yoVariableHolder.getVariable("WandererSetup", "logicPowerStateRequest");
      final YoVariable<?> motorPowerStateRequest = yoVariableHolder.getVariable("WandererSetup", "motorPowerStateRequest");
      final JButton btnPowerOn = new JButton("Power on");
      btnPowerOn.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            btnPowerOn.setEnabled(false);
            logicPowerStateRequest.setValueFromDouble(1.0);
            ThreadTools.sleep(250);
            motorPowerStateRequest.setValueFromDouble(1.0);
         }
      });
      JButton btnPowerOff = new JButton("Power off");
      btnPowerOff.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            logicPowerStateRequest.setValueFromDouble(-1);
            ThreadTools.sleep(250);
            motorPowerStateRequest.setValueFromDouble(-1);
         }
      });
      powerPanel.add(btnPowerOn);
      powerPanel.add(btnPowerOff);
      add(powerPanel);
   }
   

   private void createTable(YoVariableHolder yoVariableHolder)
   {
      DefaultTableModel tableModel = new DefaultTableModel(new String[] { "Actuator", "Motor Temperature", "MCB Temperature 0", "MCB Temperture 1", "Packet Drops", "Motor angle" }, WandererActuator.values.length);
      table = new JTable(tableModel);
      table.setFillsViewportHeight(true);
      table.getColumn("Motor Temperature").setCellRenderer(new WarningRenderer());
      table.getColumn("MCB Temperature 0").setCellRenderer(new WarningRenderer(80.0));
      table.getColumn("MCB Temperture 1").setCellRenderer(new WarningRenderer(80.0));
      table.getColumn("Packet Drops").setCellRenderer(new WarningRenderer(100.0));

      int row = 0, col = 0;
      for (WandererActuator actuator : WandererActuator.values)
      {
         motorEncoders.put(actuator, yoVariableHolder.getVariable(actuator.getName(), actuator.getName() + "MotorEncoderPosition"));
         motorTemperatures.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MotorTemperature"));
         mcbTemperatures1.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MCBTemperature1"));
         mcbTemperatures2.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MCBTemperature2"));
         consecutivePacketDropCount.put(actuator, yoVariableHolder.getVariable(actuator.getName(), actuator.getName() + "ConsecutivePacketDropCount"));

         table.setValueAt(actuator.getName(), row, col);

         col++;

         table.setValueAt(Double.NaN , row, col);

         col++;

         table.setValueAt(Double.NaN, row, col);

         col++;

         table.setValueAt(Double.NaN, row, col);

         col++;
         
         table.setValueAt(Double.NaN, row, col);

         col++;

         table.setValueAt(Double.NaN, row, col);

         col = 0;
         row++;
      }

      JScrollPane tableScroller = new JScrollPane(table);
      add(tableScroller);
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      int row = 0;
      for (WandererActuator actuator : WandererActuator.values)
      {
         table.setValueAt(String.format("%.1f", motorTemperatures.get(actuator).getValueAsDouble()), row, 1);
         table.setValueAt(String.format("%.1f", mcbTemperatures1.get(actuator).getValueAsDouble()), row, 2);
         table.setValueAt(String.format("%.1f", mcbTemperatures2.get(actuator).getValueAsDouble()), row, 3);
         table.setValueAt(String.format("%.1f", consecutivePacketDropCount.get(actuator).getValueAsDouble()), row, 4);
         table.setValueAt(String.format("%.3f", motorEncoders.get(actuator).getValueAsDouble()), row, 5);
         row++;
      }

   }

   @Override
   public void play(double realTimeRate)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void stop()
   {
      // TODO Auto-generated method stub

   }

   private class WarningRenderer extends DefaultTableCellRenderer implements TableCellRenderer
   {

      private static final long serialVersionUID = -8042596150349794691L;
      private double max_value;

      public WarningRenderer()
      {
         super();
         setFont(new Font(getFont().getName(), Font.BOLD, getFont().getSize()));
         setHorizontalAlignment(JLabel.CENTER);
         this.max_value = 100.0;
      }
      
      public WarningRenderer(double max_value)
      {
         super();
         setFont(new Font(getFont().getName(), Font.BOLD, getFont().getSize()));
         setHorizontalAlignment(JLabel.CENTER);
         this.max_value = 80.0;
      }
      
      public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column)
      {  
         Component c = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
         if(!(value instanceof Double))
         {
            if(Double.valueOf((String)value) > 1.0 * max_value)
            {
               c.setBackground(Color.RED);            
            }
            else if (Double.valueOf((String) value) > 0.8 * max_value)
            {
               c.setBackground(Color.ORANGE);
            }
            else if (Double.valueOf((String) value) > 0.6 * max_value)
            {
               c.setBackground(Color.GREEN.darker());
            }
            else
            {
               c.setBackground(Color.GREEN);
            }
         
         }
         else
         {
            c.setBackground(Color.GRAY);
         }

         setValue(value);
         return c;
      }
   }
   
}
