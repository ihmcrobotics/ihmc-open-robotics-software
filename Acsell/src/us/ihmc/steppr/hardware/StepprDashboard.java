package us.ihmc.steppr.hardware;

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.EnumMap;

import javax.swing.BoxLayout;
import javax.swing.DefaultCellEditor;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.UIManager;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableCellRenderer;

import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import com.yobotics.simulationconstructionset.PlaybackListener;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class StepprDashboard extends JPanel implements PlaybackListener
{
   private static final long serialVersionUID = -5987765792397743483L;

   private JTable table;

   private final EnumMap<StepprActuator, YoVariable> motorTemperatures = new EnumMap<>(StepprActuator.class);
   private final EnumMap<StepprActuator, YoVariable> motorEncoders = new EnumMap<>(StepprActuator.class);

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

      createTable(yoVariableHolder);

   }

   private void createLogicButtons(YoVariableHolder yoVariableHolder)
   {
      JPanel logicPanel = new JPanel();
      logicPanel.setLayout(new BoxLayout(logicPanel, BoxLayout.X_AXIS));
      final YoVariable logicPowerStateRequest = yoVariableHolder.getVariable("StepprSetup", "logicPowerStateRequest");
      JButton logicPowerOn = new JButton("Logic power on");
      logicPowerOn.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            logicPowerStateRequest.setValueFromDouble(1.0);
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
      final YoVariable motorPowerStateRequest = yoVariableHolder.getVariable("StepprSetup", "motorPowerStateRequest");
      JButton motorPowerOn = new JButton("Motor power on");
      motorPowerOn.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            motorPowerStateRequest.setValueFromDouble(1.0);
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
      DefaultTableModel tableModel = new DefaultTableModel(new String[] { "Actuator", "Motor Temperature", "<", "q", ">" }, StepprActuator.values.length);
      table = new JTable(tableModel);
      table.setFillsViewportHeight(true);
      table.getColumn("<").setCellRenderer(new NudgeRenderer(true));
      table.getColumn("<").setCellEditor(new NudgeEditor(-1));
      table.getColumn(">").setCellRenderer(new NudgeRenderer(false));
      table.getColumn(">").setCellEditor(new NudgeEditor(1));

      int row = 0, col = 0;
      for (StepprActuator actuator : StepprActuator.values)
      {
         YoVariable<?> nudgeVariable = yoVariableHolder.getVariable("StepprSetup", actuator.getName() + "Nudge");
         motorEncoders.put(actuator, yoVariableHolder.getVariable(actuator.getName(), actuator.getName() + "MotorEncoderPosition"));
         motorTemperatures.put(actuator, yoVariableHolder.getVariable(actuator.getName() + ".SlowSensors", actuator.getName() + "MotorTemperature"));

         table.setValueAt(actuator.getName(), row, col);

         col++;

         table.setValueAt(0, row, col);

         col++;

         table.setValueAt("<", row, col);
         ((NudgeEditor) table.getCellEditor(row, col)).setYoVariable(row, nudgeVariable);

         col++;

         table.setValueAt(0, row, col);

         col++;

         table.setValueAt(">", row, col);
         ((NudgeEditor) table.getCellEditor(row, col)).setYoVariable(row, nudgeVariable);

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
         table.setValueAt(String.format("%.3f", motorEncoders.get(actuator).getValueAsDouble()), row, 3);
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

   private class NudgeRenderer extends JButton implements TableCellRenderer
   {
      private final boolean decrease;

      public NudgeRenderer(boolean decrease)
      {
         this.decrease = decrease;
         setOpaque(true);
      }

      public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column)
      {
         setForeground(table.getForeground());
         setBackground(UIManager.getColor("Button.background"));
         if (decrease)
         {
            setText("<");
         }
         else
         {
            setText(">");
         }

         return this;
      }
   }

   private class NudgeEditor extends DefaultCellEditor
   {
      protected JButton button;

      private final int direction;
      private boolean isPushed;
      private int rowClicked;

      private YoVariable[] yoVariables = new YoVariable[StepprActuator.values.length];

      public NudgeEditor(int direction)
      {
         super(new JCheckBox());
         this.direction = direction;
         button = new JButton();
         button.setOpaque(true);
         button.addActionListener(new ActionListener()
         {
            public void actionPerformed(ActionEvent e)
            {
               fireEditingStopped();
            }
         });
      }

      public void setYoVariable(int row, YoVariable variable)
      {
         yoVariables[row] = variable;
      }

      public Component getTableCellEditorComponent(JTable table, Object value, boolean isSelected, int row, int column)
      {
         if (isSelected)
         {
            button.setForeground(table.getSelectionForeground());
            button.setBackground(table.getSelectionBackground());
         }
         else
         {
            button.setForeground(table.getForeground());
            button.setBackground(table.getBackground());
         }

         isPushed = true;

         rowClicked = row;

         return button;
      }

      public Object getCellEditorValue()
      {
         if (isPushed)
         {
            yoVariables[rowClicked].setValueFromDouble(direction);
         }

         isPushed = false;

         return yoVariables[rowClicked];
      }

      public boolean stopCellEditing()
      {
         isPushed = false;

         return super.stopCellEditing();
      }

      protected void fireEditingStopped()
      {
         super.fireEditingStopped();
      }
   }
}
