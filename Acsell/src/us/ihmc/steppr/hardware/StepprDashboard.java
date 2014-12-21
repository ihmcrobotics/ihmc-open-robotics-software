package us.ihmc.steppr.hardware;

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

import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class StepprDashboard extends JPanel implements PlaybackListener
{
   private static final long serialVersionUID = -5987765792397743483L;

   private JTable table;

   private final EnumMap<StepprActuator, YoVariable<?>> motorTemperatures = new EnumMap<>(StepprActuator.class);
   private final EnumMap<StepprActuator, YoVariable<?>> mcbTemperatures1 = new EnumMap<>(StepprActuator.class);
   private final EnumMap<StepprActuator, YoVariable<?>> mcbTemperatures2 = new EnumMap<>(StepprActuator.class);
   private final EnumMap<StepprActuator, YoVariable<?>> motorEncoders = new EnumMap<>(StepprActuator.class);

   public static void createDashboard(final SimulationConstructionSet scs, YoVariableHolder registry)
   {
      StepprDashboard stepprDashboard = new StepprDashboard(registry);
      scs.addExtraJpanel(stepprDashboard, "Dashboard");
      scs.attachPlaybackListener(stepprDashboard);
      
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
   
   private StepprDashboard(YoVariableHolder yoVariableHolder)
   {
      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

      createLogicButtons(yoVariableHolder);
      createMotorButtons(yoVariableHolder);
      createCalibrationButtons(yoVariableHolder);

      createTable(yoVariableHolder);

   }

   private void createCalibrationButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel calibrationPanel = new JPanel();
      calibrationPanel.setLayout(new BoxLayout(calibrationPanel, BoxLayout.X_AXIS));
      final YoVariable<?> updateOffsets = yoVariableHolder.getVariable("Steppr", "updateOffsets");
      final YoVariable<?> tareSensorsVar = yoVariableHolder.getVariable("Steppr", "tareSensors");
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

   private void createLogicButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel logicPanel = new JPanel();
      logicPanel.setLayout(new BoxLayout(logicPanel, BoxLayout.X_AXIS));
      final YoVariable<?> logicPowerStateRequest = yoVariableHolder.getVariable("StepprSetup", "logicPowerStateRequest");
      final JButton logicPowerOn = new JButton("Logic power on");
      logicPowerOn.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            logicPowerStateRequest.setValueFromDouble(1.0);
            logicPowerOn.setEnabled(false);
         }
      });
      JButton logicPowerOff = new JButton("Logic power off");
      logicPowerOff.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            logicPowerStateRequest.setValueFromDouble(-1);
         }
      });
      logicPanel.add(logicPowerOn);
      logicPanel.add(logicPowerOff);
      add(logicPanel);
   }

   private void createMotorButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel motorPanel = new JPanel();
      motorPanel.setLayout(new BoxLayout(motorPanel, BoxLayout.X_AXIS));
      final YoVariable<?> motorPowerStateRequest = yoVariableHolder.getVariable("StepprSetup", "motorPowerStateRequest");
      final JButton motorPowerOn = new JButton("Motor power on");
      motorPowerOn.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            motorPowerStateRequest.setValueFromDouble(1.0);
            motorPowerOn.setEnabled(false);
         }
      });
      JButton motorPowerOff = new JButton("Motor power off");
      motorPowerOff.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            motorPowerStateRequest.setValueFromDouble(-1);
         }
      });
      motorPanel.add(motorPowerOn);
      motorPanel.add(motorPowerOff);
      add(motorPanel);
   }

   private void createTable(YoVariableHolder yoVariableHolder)
   {
      DefaultTableModel tableModel = new DefaultTableModel(new String[] { "Actuator", "Motor Temperature", "MCB Temperature 0", "MCB Temperture 1", "Motor angle" }, StepprActuator.values.length);
      table = new JTable(tableModel);
      table.setFillsViewportHeight(true);
      table.getColumn("Motor Temperature").setCellRenderer(new WarningRenderer());
      table.getColumn("MCB Temperature 0").setCellRenderer(new WarningRenderer());
      table.getColumn("MCB Temperture 1").setCellRenderer(new WarningRenderer());

      int row = 0, col = 0;
      for (StepprActuator actuator : StepprActuator.values)
      {
         YoVariable<?> nudgeVariable = yoVariableHolder.getVariable("StepprSetup", actuator.getName() + "Nudge");
         motorEncoders.put(actuator, yoVariableHolder.getVariable(actuator.getName(), actuator.getName() + "MotorEncoderPosition"));
         motorTemperatures.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MotorTemperature"));
         mcbTemperatures1.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MCBTemperature1"));
         mcbTemperatures2.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MCBTemperature2"));

         table.setValueAt(actuator.getName(), row, col);

         col++;

         table.setValueAt(Double.NaN , row, col);

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
      for (StepprActuator actuator : StepprActuator.values)
      {
         table.setValueAt(String.format("%.1f", motorTemperatures.get(actuator).getValueAsDouble()), row, 1);
         table.setValueAt(String.format("%.1f", mcbTemperatures1.get(actuator).getValueAsDouble()), row, 2);
         table.setValueAt(String.format("%.1f", mcbTemperatures2.get(actuator).getValueAsDouble()), row, 3);
         table.setValueAt(String.format("%.3f", motorEncoders.get(actuator).getValueAsDouble()), row, 4);
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

      public WarningRenderer()
      {
         super();
         setFont(new Font(getFont().getName(), Font.BOLD, getFont().getSize()));
         setHorizontalAlignment(JLabel.CENTER);
      }
      
      public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column)
      {  
         Component c = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
         if(!(value instanceof Double))
         {
            if(Double.valueOf((String)value) > 100.0)
            {
               c.setBackground(Color.RED);            
            }
            else if (Double.valueOf((String) value) > 80.0)
            {
               c.setBackground(Color.ORANGE);
            }
            else if (Double.valueOf((String) value) > 60.0)
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
