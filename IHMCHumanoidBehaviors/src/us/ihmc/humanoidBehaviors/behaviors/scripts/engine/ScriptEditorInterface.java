package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusAdapter;
import java.awt.event.FocusEvent;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

import javax.swing.AbstractAction;
import javax.swing.ActionMap;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.InputMap;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRootPane;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.plaf.basic.BasicArrowButton;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.JTableHeader;

import us.ihmc.commons.Conversions;
import us.ihmc.robotics.EpsilonComparable;

public class ScriptEditorInterface
{
   public final JFrame jFrame;

   private final JTable table;
   private final JTable table2;
   private final DefaultTableModel model;
   private final DefaultTableModel model2;
   private JCheckBox table1check = new JCheckBox("table 1");
   private JCheckBox table2check = new JCheckBox("table 2");
   private JPanel outputArea1 = new JPanel();
   private JPanel outputArea2 = new JPanel();
   private JPanel outputXMLFile = new JPanel();
   private JPanel loadInfoArea = new JPanel();
   private final LoadScriptListener loadScriptListener = new LoadScriptListener();

   private final TextTool textTools = new TextTool();
   private final JTextField saveTableDataName;
   private final JTextField setDeltaTime;
   private final JTextField setEpsilon;

   private final JComboBox<String> scriptTypeSelector;
   private final JTextArea loadInfo;
   private final JLabel frameLabel;
   private final JTabbedPane tabbedPane;
   private long initialTimeOffsetNano;

   private final String loadInfoText;
   private final String loadInstructionText;
   private final String loadErrorText;
   private int loadCounter;

   private File scriptFile1;
   private JButton compareScripts;
   private JButton openXML;

   private final ArrayList<ScriptObject> deletedScripts = new ArrayList<ScriptObject>();
   private final ArrayList<Integer> deletedScriptsPos = new ArrayList<Integer>();
   private final ArrayList<Long> deletedScriptsTime = new ArrayList<Long>();
   private final String[] scriptTypeSelectorString =
   {
      "Chest Orientation", "Foot Load Bearing", "Foot Pose", "Hand F Closed", "Hand Pose", "Hand Load Bearing", "Hand Opened", "Hand T&F Closed",
      "HeadOrientationPacket", "Pelvis Pose"
   };

   private ArrayList<ScriptObject> tableData = new ArrayList<ScriptObject>();
   private ArrayList<ScriptObject> tableData2 = new ArrayList<ScriptObject>();

   public static void main(String[] args)
   {
      new ScriptEditorInterface();
   }

   @SuppressWarnings("serial")
   public ScriptEditorInterface()
   {
      Font font = new Font("Courier", Font.BOLD, 12);
      loadCounter = 0;
      jFrame = new JFrame("Script Editor 2013 --- 2.0");
      JPanel buttonPanel = new JPanel();
      JPanel buttonPanelLeft = new JPanel();
      JPanel buttonPanelSave = new JPanel();
      JPanel buttonPanelEdit = new JPanel();
      buttonPanelLeft.setLayout(new BoxLayout(buttonPanelLeft, BoxLayout.Y_AXIS));

      saveTableDataName = new JTextField("enter unique name", 60);
      setDeltaTime = new JTextField(4);
      setEpsilon = new JTextField(4);
      setEpsilon.setText("0.05");

      loadInfo = new JTextArea(26, 60);
      loadInfo.setFont(font);
      loadInfo.setLineWrap(true);
      loadInfo.setBackground(new Color(214, 228, 225));
      loadInfo.setEditable(false);
      loadInfo.setBorder(BorderFactory.createMatteBorder(4, 2, 3, 3, new Color(140, 156, 154)));
      loadInfoText = " Scripts loaded in sequence: \n\n ";
      loadInstructionText =
         "Instructions: \n\n W = Move Object Up || S = Move Object Down \n\n C = Copy Object || Del = Delete Object \n\n F5 = Refresh Table || Z = Undo Delete row \n\n P = Copy Right to Left [Edit Scripts in Table 1] \n\n \n\n";
      loadErrorText = " ";
      loadInfo.append(loadInstructionText);
      loadInfo.append(loadInfoText);
      loadInfo.append(loadErrorText);
      loadInfo.setToolTipText("Info on the loaded scripts in sequence");

      tabbedPane = new JTabbedPane();

      buttonPanel.setBackground(new Color(214, 228, 225));
      buttonPanel.setBorder(BorderFactory.createMatteBorder(4, 2, 3, 3, new Color(140, 156, 154)));
      buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.X_AXIS));

      outputArea1.setBackground(new Color(140, 156, 154));
      outputArea2.setBackground(new Color(140, 156, 154));
      outputXMLFile.setBackground(new Color(140, 156, 154));
      outputArea1.setToolTipText("Table 1");
      outputArea2.setToolTipText("Table 2");
      outputXMLFile.setToolTipText("Edit File");

      // outputTextArea.setSize(600, 300);
      outputXMLFile.setBackground(new Color(140, 156, 154));
      outputXMLFile.add(textTools);

      jFrame.getContentPane().add(BorderLayout.SOUTH, buttonPanel);
      jFrame.getContentPane().add(BorderLayout.CENTER, loadInfoArea);
      jFrame.getContentPane().add(BorderLayout.WEST, outputArea1);
      jFrame.getContentPane().add(BorderLayout.EAST, outputArea2);

      table1check.setToolTipText("Check to load to table 1");
      table1check.addActionListener(new checkListener());

      table2check.setToolTipText("Check to load to table 2");
      table2check.addActionListener(new checkListener());

      JButton scriptLoad = new JButton("Load Script");
      scriptLoad.setToolTipText("Load a Script in the Script Editor. Multiple scripts can be loaded");
      scriptLoad.addActionListener(loadScriptListener);

      JButton clearTableData = new JButton("Clear Table");
      clearTableData.setToolTipText("Delete all the scripts in selected table");
      clearTableData.addActionListener(new DeleteTableListener());

      JButton clearTablesData = new JButton("Clear Both Tables");
      clearTablesData.setToolTipText("Delete all the scripts in both tables");
      clearTablesData.addActionListener(new DeleteAllTablesListener());

      JButton deleteSelectedRows = new JButton("Delete Selected");
      deleteSelectedRows.setToolTipText("Delete the selected row in the table (Delete/Backspace)");
      deleteSelectedRows.addActionListener(new DeleteSelectedRows());

      JButton saveTableData = new JButton("Save");
      saveTableData.setToolTipText("Save the current table data in to a script");
      saveTableData.addActionListener(new SaveTableListener());

      compareScripts = new JButton("Compare");
      compareScripts.setToolTipText("Compare the equivalence of two scripts");
      compareScripts.addActionListener(new CompareScripts());

      JButton deleteAllPauses = new JButton("Delete Pauses/ 0Footsteps");
      deleteAllPauses.setToolTipText("Delete all the pauses and 0 Footsteps at once in the whole table data");
      deleteAllPauses.addActionListener(new DeleteAllPauses());

      BasicArrowButton moveRowUp = new BasicArrowButton(BasicArrowButton.NORTH);
      moveRowUp.setBackground(Color.ORANGE);
      moveRowUp.setToolTipText("Move selected Row up (W)");
      moveRowUp.addActionListener(new MoveRowUp());

      BasicArrowButton moveRowDown = new BasicArrowButton(BasicArrowButton.SOUTH);
      moveRowDown.setBackground(Color.ORANGE);
      moveRowDown.setToolTipText("Move selected Row down (S)");
      moveRowDown.addActionListener(new MoveRowDown());

      JButton setNewDeltaTimes = new JButton("Set delta time [s]");
      setNewDeltaTimes.setToolTipText("sets the delta time for all the chosen scriptObjects");
      setNewDeltaTimes.addActionListener(new SetNewDeltaTimes());

      JButton refreshTableData = new JButton("Refresh");
      refreshTableData.setToolTipText("refresh the TableData");
      refreshTableData.addActionListener(new RefreshTableData());

      openXML = new JButton("Show XML File");
      openXML.setToolTipText("Edit scripts right here");
      openXML.addActionListener(new OpenXML());

      scriptTypeSelector = new JComboBox<String>(scriptTypeSelectorString);

      frameLabel = new JLabel("No reference frame");
      frameLabel.setToolTipText("Shows the current reference frame of the loaded script. Only scripts with the same reference frame can be loaded");

      setDeltaTime.addFocusListener(new FocusAdapter()
      {
         @Override
         public void focusGained(FocusEvent fEvt)
         {
            JTextField tField = (JTextField) fEvt.getSource();
            tField.selectAll();
         }
      });

      saveTableDataName.addFocusListener(new FocusAdapter()
      {
         @Override
         public void focusGained(FocusEvent fEvt)
         {
            JTextField tField = (JTextField) fEvt.getSource();
            tField.selectAll();
         }
      });

      model = new DefaultTableModel(new Object[] { "#", "ScriptObject", "DeltaTime [s]", "Time [s]" }, 0);
      table = new JTable(model);
      JTableHeader header = table.getTableHeader();
      header.setBackground(Color.LIGHT_GRAY);
      table.getColumn("ScriptObject").setPreferredWidth(390);
      table.getColumn("#").setPreferredWidth(50);
      table.getColumn("DeltaTime [s]").setPreferredWidth(115);
      table.getColumn("Time [s]").setPreferredWidth(100);
      JScrollPane pane = new JScrollPane(table);

      model2 = new DefaultTableModel(new Object[] { "#", "ScriptObject", "DeltaTime [s]", "Time [s]" }, 0);
      table2 = new JTable(model2);
      JTableHeader header2 = table2.getTableHeader();
      header2.setBackground(Color.LIGHT_GRAY);
      table2.getColumn("ScriptObject").setPreferredWidth(390);
      table2.getColumn("#").setPreferredWidth(50);
      table2.getColumn("DeltaTime [s]").setPreferredWidth(115);
      table2.getColumn("Time [s]").setPreferredWidth(100);
      JScrollPane pane2 = new JScrollPane(table2);

      table2.setAutoResizeMode(JTable.AUTO_RESIZE_ALL_COLUMNS); // AUTO_RESIZE_OFF);
      table2.getTableHeader().setReorderingAllowed(false);

      outputArea1.add(pane);
      outputArea2.add(pane2);
      loadInfoArea.add(loadInfo);
      buttonPanel.add(table1check);
      buttonPanel.add(table2check);

      buttonPanelSave.add(scriptLoad);
      buttonPanelSave.add(saveTableDataName);
      buttonPanelSave.add(frameLabel);
      buttonPanelSave.add(saveTableData);
      buttonPanelSave.add(setEpsilon);
      buttonPanelSave.add(compareScripts);
      buttonPanelSave.add(openXML);

      buttonPanelEdit.add(deleteSelectedRows);
      buttonPanelEdit.add(deleteAllPauses);
      buttonPanelEdit.add(scriptTypeSelector);
      buttonPanelEdit.add(setDeltaTime);
      buttonPanelEdit.add(setNewDeltaTimes);
      buttonPanelEdit.add(refreshTableData);
      buttonPanelEdit.add(clearTableData);
      buttonPanelEdit.add(clearTablesData);

      buttonPanel.add(tabbedPane);
      tabbedPane.addTab("<html><body leftmargin=15 topmargin=8 marginwidth=20 marginheight=1>File</body></html>", buttonPanelSave);
      tabbedPane.addTab("<html><body leftmargin=15 topmargin=8 marginwidth=20 marginheight=1>Edit Script</body></html>", buttonPanelEdit);

      buttonPanelLeft.setSize(200, 200);
      buttonPanelLeft.add(moveRowUp, BorderLayout.EAST);
      buttonPanelLeft.add(moveRowDown, BorderLayout.EAST);

      pane.setVisible(true);
      jFrame.setSize(1390, 534);
      jFrame.setResizable(false);
      jFrame.setLocationRelativeTo(null);
      jFrame.setUndecorated(true); // delete the border for OLDskool look
      jFrame.getRootPane().setWindowDecorationStyle(JRootPane.PLAIN_DIALOG);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      jFrame.setVisible(true);

      InputMap inputMap = table.getInputMap(JTable.WHEN_FOCUSED);
      InputMap inputMap2 = table2.getInputMap(JTable.WHEN_FOCUSED);
      InputMap im2 = table2.getInputMap();
      InputMap im = table.getInputMap();
      ActionMap actionMap = table.getActionMap();
      ActionMap actionMap2 = table2.getActionMap();

      inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE, 0), "DeleteRow");
      inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_BACK_SPACE, 0), "DeleteRow");
      inputMap2.put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE, 0), "DeleteRow");

      actionMap.put("DeleteRow", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            // TODO: integrate with DeleteSelectedRows class
            int[] rowIndex = table.getSelectedRows();
            int counter = 0;
            for (int i = 0; i < rowIndex.length; i++)
            {
               deleteSelectedRows(rowIndex[i] - counter);
               counter++;
            }

            table.getSelectionModel().setSelectionInterval(rowIndex[0] - 1, rowIndex[0] - 1);
            table.scrollRectToVisible(new Rectangle(table.getCellRect(rowIndex[0] + 1, 0, true)));

            if (rowIndex[0] == 0)
            {
               table.getSelectionModel().setSelectionInterval(0, 0);
            }
         }
      });

      inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_Z, 0), "UndoDeleteRow");
      actionMap.put("UndoDeleteRow", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            undoDeleteSelectedRows();
         }
      });

      im.put(KeyStroke.getKeyStroke(KeyEvent.VK_W, 0), "MoveRowUp");
      actionMap.put("MoveRowUp", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            moveRowUp();
         }
      });

      im.put(KeyStroke.getKeyStroke(KeyEvent.VK_S, 0), "MoveRowDown");
      actionMap.put("MoveRowDown", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            moveRowDown();
         }
      });

      im.put(KeyStroke.getKeyStroke(KeyEvent.VK_C, 0), "CopyRow");
      actionMap.put("CopyRow", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            copyRow();
         }
      });

      im2.put(KeyStroke.getKeyStroke(KeyEvent.VK_P, 0), "CopyRowOver");
      actionMap2.put("CopyRowOver", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            copyRowOver();
         }
      });

      im.put(KeyStroke.getKeyStroke(KeyEvent.VK_F5, 0), "refreshTableData");
      actionMap.put("refreshTableData", new AbstractAction()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            refreshTableData();
         }
      });

   }

   private class checkListener implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         checkWhichBox(table1check, table2check);
      }
   }

   private class SetNewDeltaTimes implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         int selected = scriptTypeSelector.getSelectedIndex();

         for (int i = 0; i < tableData.size(); i++)
         {
            if (tableData.get(i).getScriptObject().toString().contains(scriptTypeSelectorString[selected]))
            {
               long time = tableData.get(i).getTimeStamp();
               double deltaTimeDes = Double.parseDouble(setDeltaTime.getText()) * 1000000000;
               long deltaTimeDesired = (long) deltaTimeDes;
               tableData.get(i + 1).setTimeStamp(time + deltaTimeDesired);

               for (int j = i + 2; j < tableData.size(); j++)
               {
                  long deltaTime = Conversions.secondsToNanoSeconds(Double.parseDouble(model.getValueAt(j - 1, 2).toString()));
                  tableData.get(j).setTimeStamp(tableData.get(j - 1).getTimeStamp() + deltaTime);
               }

               loadScriptListener.setData(tableData);
            }
         }
      }
   }

   private class RefreshTableData implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         refreshTableData();
      }
   }

   private class OpenXML implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         openXML();
      }
   }

   private class LoadScriptListener implements ActionListener
   {
      private ArrayList<ScriptObject> script;
      private File scriptFile;

      @Override
      public void actionPerformed(ActionEvent event)
      {
         if (checkWhichBox(table1check, table2check) == 0)
         {
            loadInfo.append("                  NO TABLE SELECTED!            ");
         }

         if (checkWhichBox(table1check, table2check) > 0)
         {
            loadCounter++;
            scriptFile = ScriptFileSelector.getScriptFileFromUserSelection(ScriptEngineSettings.extension);

            // We will no longer check to see if the two scripts are in the same frame
            // checkFrame(scriptFile);

            if (checkWhichBox(table1check, table2check) == 1)
            {
               loadInfo.append(loadCounter + ") Table 1:  " + scriptFile.getName() + "\n\n");
               scriptFile1 = scriptFile;
            }

            if (checkWhichBox(table1check, table2check) == 2)
            {
               loadInfo.append(loadCounter + ") Table 2:  " + scriptFile.getName() + "\n\n");
            }

            if (checkWhichBox(table1check, table2check) == 3)
            {
               loadInfo.append(loadCounter + ") Table 1 & 2:  " + scriptFile.getName() + "\n\n");
               scriptFile1 = scriptFile;
            }

            try
            {
               ScriptFileLoader loader = new ScriptFileLoader(scriptFile);
               script = loader.readIntoList();
               loader.close();
               appendData(script);

               if (checkWhichBox(table1check, table2check) == 1)
               {
                  if (tableData.size() > 1)
                     tableData.remove(tableData.size() - 1);
                  tableData.addAll(script);
               }

               if (checkWhichBox(table1check, table2check) == 2)
               {
                  if (tableData2.size() > 1)
                     tableData2.remove(tableData2.size() - 1);
                  tableData2.addAll(script);
               }

               if (checkWhichBox(table1check, table2check) == 3)
               {
                  if (tableData.size() > 1)
                     tableData.remove(tableData.size() - 1);
                  tableData.addAll(script);
                  if (tableData2.size() > 1)
                     tableData2.remove(tableData2.size() - 1);
                  tableData2.addAll(script);
               }

               table1check.setSelected(false);
               outputArea2.setBackground(new Color(140, 156, 154));
               table2check.setSelected(false);
               outputArea1.setBackground(new Color(140, 156, 154));
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }

         textTools.openTableData(scriptFile1.getAbsolutePath());
      }

      public void appendData(ArrayList<ScriptObject> tableData)
      {
         DefaultTableModel whichModel = model;
         if (checkWhichBox(table1check, table2check) == 1)
         {
            // Do nothing
         }

         else if (checkWhichBox(table1check, table2check) == 2)
         {
            whichModel = model2;
         }

         else if (checkWhichBox(table1check, table2check) == 3)
         {
            if (model2.getRowCount() > 0)
            {
               model2.setRowCount(model2.getRowCount() - 1);
            }

            if (model2.getRowCount() == 0)
               initialTimeOffsetNano = tableData.get(0).getTimeStamp();

            int rowOffset = model2.getRowCount();

            for (int i = 0; i < tableData.size(); i++)
            {
               ScriptObject scriptObject = tableData.get(i);
               double time = Conversions.nanoSecondstoSeconds(tableData.get(i).getTimeStamp());

               double deltaT = (i == tableData.size() - 1) ? 0.0 : Conversions.nanoSecondstoSeconds((long) (tableData.get(i + 1).getTimeStamp() - tableData
                     .get(i).getTimeStamp()));
               model2.addRow(new Object[] { rowOffset + i + 1, scriptObject.toString(), deltaT, time });
            }
         }

         if (whichModel.getRowCount() > 0)
         {
            whichModel.setRowCount(whichModel.getRowCount() - 1);
         }

         if (whichModel.getRowCount() == 0)
            initialTimeOffsetNano = tableData.get(0).getTimeStamp();

         int rowOffset1 = whichModel.getRowCount();

         for (int i = 0; i < tableData.size(); i++)
         {
            ScriptObject scriptObject = tableData.get(i);
            double time = Conversions.nanoSecondstoSeconds(tableData.get(i).getTimeStamp());

            double deltaT = (i == tableData.size() - 1) ? 0.0 : Conversions.nanoSecondstoSeconds((long) (tableData.get(i + 1).getTimeStamp() - tableData.get(i)
                  .getTimeStamp()));

            whichModel.addRow(new Object[] { rowOffset1 + i + 1, scriptObject.toString(), deltaT, time });
         }
      }

      public void setData(ArrayList<ScriptObject> tableData)
      {
         model.setRowCount(0);

         if (tableData.size() != 0)
         {
            initialTimeOffsetNano = tableData.get(0).getTimeStamp();
            appendData(tableData);
         }
      }

      public void removeRow(int row)
      {
         model.removeRow(row);
      }
   }

   private class DeleteAllTablesListener implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         deleteAllTables();
      }
   }

   private class DeleteTableListener implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         deleteAll();
      }
   }

   private class DeleteSelectedRows implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         int[] rowIndex = table.getSelectedRows();
         int counter = 0;
         for (int i = 0; i < rowIndex.length; i++)
         {
            deleteSelectedRows(rowIndex[i] - counter);
            counter++;
         }

         table.getSelectionModel().setSelectionInterval(rowIndex[0] - 1, rowIndex[0] - 1);
         table.scrollRectToVisible(new Rectangle(table.getCellRect(rowIndex[0] + 1, 0, true)));

         if (rowIndex[0] == 0)
         {
            table.getSelectionModel().setSelectionInterval(0, 0);
         }
      }
   }

   private class SaveTableListener implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         setTimeStampsFromDeltaTimes();
         Path saveFilePath = Paths.get(ScriptEngineSettings.scriptSavingDirectory, saveTableDataName.getText() + ScriptEngineSettings.extension);
         ScriptFileSaver scriptFileSaver;
         try
         {
            scriptFileSaver = new ScriptFileSaver(saveFilePath, false);
            scriptFileSaver.recordList(tableData);
            scriptFileSaver.close();
            saveTableDataName.setText("Save succesfull!");
            loadInfo.setText(loadInfoText);
            loadInfo.append("Table 1's Script saved in: \n" + saveFilePath + "\n");

            ArrayList<ScriptObject> temp = tableData;
            model.setRowCount(0);
            loadScriptListener.setData(temp);
         }
         catch (IOException e)
         {
            e.printStackTrace();
            saveTableDataName.setText("Unable to save!");
         }
      }
   }

   private class CompareScripts implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         compareScripts();
      }
   }

   private class DeleteAllPauses implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         for (int i = 0; i < tableData.size(); i++)
         {
            if ((model.getValueAt(i, 1).toString()).equals("Paused = true") || model.getValueAt(i, 1).toString().equals("0 Footsteps"))
            {
               model.removeRow(i);
               tableData.remove(i);
               i -= 1;
            }
         }

         for (int i = 0; i < tableData.size(); i++)
         {
            if ((model2.getValueAt(i, 1).toString()).equals("Paused = true") || model2.getValueAt(i, 1).toString().equals("0 Footsteps"))
            {
               model2.removeRow(i);
               tableData2.remove(i);
               i -= 1;
            }
         }

         refreshTableData();
      }
   }

   private class MoveRowUp implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         moveRowUp();
      }
   }

   private class MoveRowDown implements ActionListener
   {
      @Override
      public void actionPerformed(ActionEvent event)
      {
         moveRowDown();
      }
   }

   private double checkWhichBox(JCheckBox table1check, JCheckBox table2check)
   {
      if (table1check.isSelected() && table2check.isSelected())
      {
         outputArea1.setBackground(new Color(163, 30, 57));
         outputArea2.setBackground(new Color(163, 30, 57));

         return (double) 3;
      }

      if (table1check.isSelected())
      {
         outputArea1.setBackground(new Color(163, 30, 57));
         outputArea2.setBackground(new Color(140, 156, 154));

         return (double) 1;
      }

      if (table2check.isSelected())
      {
         outputArea2.setBackground(new Color(163, 30, 57));
         outputArea1.setBackground(new Color(140, 156, 154));

         return (double) 2;
      }
      else
         outputArea1.setBackground(new Color(140, 156, 154));

      outputArea2.setBackground(new Color(140, 156, 154));

      return (double) 0;
   }

   private void setTimeStampsFromDeltaTimes()
   {
      long currentTimeStampNano = initialTimeOffsetNano;

      for (int i = 0; i < tableData.size(); i++)
      {
         tableData.get(i).setTimeStamp(currentTimeStampNano);

         double deltaT = 0.0;
         Object dtToCast = table.getValueAt(i, 2);
         if (dtToCast instanceof String)
            deltaT = Double.parseDouble((String) dtToCast);
         else if (dtToCast instanceof Double)
            deltaT = (Double) dtToCast;
         currentTimeStampNano += Conversions.secondsToNanoSeconds(deltaT);
      }
   }

   public void deleteSelectedRows(int rowIndex)
   {
      long timeDelta = 0;

      if (rowIndex == table.getRowCount() - 1)
      {
         return;
      }

      timeDelta = Conversions.secondsToNanoSeconds(Double.parseDouble(model.getValueAt(rowIndex, 2).toString()));
      deletedScripts.add(tableData.get(rowIndex));
      deletedScriptsPos.add(rowIndex);
      deletedScriptsTime.add(timeDelta);
      tableData.remove(rowIndex);
      loadScriptListener.removeRow(rowIndex);
      setTimeStamp(rowIndex, timeDelta, false);
      loadScriptListener.setData(tableData);
      if (tableData.size() == 0)
         frameLabel.setText("No reference frame");
   }

   public void moveRowUp()
   {
      int[] selectedRow = table.getSelectedRows();
      if (!(selectedRow.length > 0) || (selectedRow[selectedRow.length - 1] == 0) || (selectedRow[selectedRow.length - 1] == tableData.size() - 1)
            || (selectedRow.length > 1))
      {
         // Do nothing
      }
      else
      {
         model.moveRow(selectedRow[0], selectedRow[selectedRow.length - 1], selectedRow[0] - 1);
         table.setRowSelectionInterval(selectedRow[0] - 1, selectedRow[0] - 1); // TODO if multiple rows are selected move selection with it.
         table.getSelectionModel().setSelectionInterval(selectedRow[0] - 1, selectedRow[0] - 1);
         table.scrollRectToVisible(new Rectangle(table.getCellRect(selectedRow[0] - 1, 0, true)));

         ScriptObject toMove = tableData.get(selectedRow[0]);
         tableData.set(selectedRow[0], tableData.get(selectedRow[0] - 1));
         tableData.set(selectedRow[0] - 1, toMove);
      }
   }

   public void moveRowDown()
   {
      int[] selectedRow = table.getSelectedRows();
      if (checkWhichBox(table1check, table2check) > 1)
      {
         // Do Nothing
      }

      if (!(selectedRow.length > 0) || (selectedRow[selectedRow.length - 1] >= tableData.size() - 2) || (selectedRow.length > 1))
      {
         // Do nothing
      }
      else
      {
         model.moveRow(selectedRow[0], selectedRow[selectedRow.length - 1], selectedRow[0] + 1);
         table.setRowSelectionInterval(selectedRow[0] + 1, selectedRow[0] + 1); // TODO if multiple rows are selected move selection with it.
         table.getSelectionModel().setSelectionInterval(selectedRow[0] + 1, selectedRow[0] + 1);
         table.scrollRectToVisible(new Rectangle(table.getCellRect(selectedRow[0] + 1, 0, true)));

         ScriptObject toMove = tableData.get(selectedRow[0]);
         tableData.set(selectedRow[0], tableData.get(selectedRow[0] + 1));
         tableData.set(selectedRow[0] + 1, toMove);
      }
   }

   @SuppressWarnings("unchecked")
   public void compareScripts()
   {
      double epsilon = Double.parseDouble(setEpsilon.getText());
      boolean equals;
      int counter = 0;
      if (!(tableData.size() == tableData2.size()))
      {
         equals = false;
         counter++;
      }
      else

         for (int i = 0; i < tableData.size() - 1; i++)
         {
            Object tableDataObject1 = tableData.get(i).getScriptObject();
            Object tableDataObject2 = tableData2.get(i).getScriptObject();
            if (tableDataObject1 instanceof EpsilonComparable)
            {
               equals = ((EpsilonComparable<Object>) tableDataObject1).epsilonEquals(tableDataObject2, epsilon);

               if (!equals)
               {
                  counter++;
                  System.out.println(i + " " + tableDataObject1 + " is outside epsilon of:  " + epsilon + "\n");
               }
            }
            else
            {
               equals = tableDataObject1.equals(tableDataObject2);

               if (!equals)
               {
                  counter++;
                  compareScripts.setBackground(Color.RED);
                  loadInfo.append(i + " " + tableDataObject1 + " is outside epsilon of:  " + epsilon + "\n");
               }
            }
         }

      if (counter >= 1)
      {
         compareScripts.setBackground(Color.RED);
      }
      else
      {
         compareScripts.setBackground(Color.GREEN);
      }
   }

   public void copyRow()
   {
      int[] selectedRow = table.getSelectedRows();
      if (!(selectedRow.length > 0))
      {
         // Do nothing
      }
      else
      {
         long timeDelta = Conversions.secondsToNanoSeconds(Double.parseDouble(model.getValueAt(selectedRow[0], 2).toString()));

         ScriptObject selectedRowinTable = tableData.get(selectedRow[0]);
         ScriptObject cloneObject = new ScriptObject(selectedRowinTable.getTimeStamp() + timeDelta, selectedRowinTable.getScriptObject());
         tableData.add(selectedRow[0] + 1, cloneObject);

         for (int i = selectedRow[0] + 2; i < tableData.size(); i++)
         {
            long time = tableData.get(i).getTimeStamp();
            tableData.get(i).setTimeStamp(time + timeDelta);
         }

         loadScriptListener.setData(tableData);
         refreshTableData();
         table.getSelectionModel().setSelectionInterval(selectedRow[0] + 1, selectedRow[0] + 1);
         table.scrollRectToVisible(new Rectangle(table.getCellRect(selectedRow[0] + 1, 0, true)));
      }
   }

   public void copyRowOver()
   {
      int[] selectedRow = table2.getSelectedRows();
      long setTimeForCopy = 0;
      if (!(selectedRow.length > 0))
      {
         // Do nothing
      }
      else
      {
         if (selectedRow[0] == tableData2.size() - 1)
            return;

         long timeDeltaToCopy = Conversions.secondsToNanoSeconds(Double.parseDouble(model2.getValueAt(selectedRow[0], 2).toString()));
         if (selectedRow[0] == 0)
         {
            setTimeForCopy = tableData2.get(selectedRow[0]).getTimeStamp();
         }
         else
         {
            long timeDelta = Conversions.secondsToNanoSeconds(Double.parseDouble(model.getValueAt(selectedRow[0] - 1, 2).toString()));
            setTimeForCopy = tableData.get(selectedRow[0] - 1).getTimeStamp() + timeDelta;
         }

         ScriptObject selectedRowinTable = tableData2.get(selectedRow[0]);
         ScriptObject cloneObject = new ScriptObject(setTimeForCopy, selectedRowinTable.getScriptObject());
         tableData.add(selectedRow[0], cloneObject);

         for (int i = selectedRow[0] + 1; i < tableData.size(); i++)
         {
            long time = tableData.get(i).getTimeStamp();
            tableData.get(i).setTimeStamp(time + timeDeltaToCopy);
         }

         loadScriptListener.setData(tableData);
         refreshTableData();
         table.getSelectionModel().setSelectionInterval(selectedRow[0] + 1, selectedRow[0] + 1);
         table.scrollRectToVisible(new Rectangle(table.getCellRect(selectedRow[0] + 1, 0, true)));
      }
   }

   public void refreshTableData()
   {
      setTimeStampsFromDeltaTimes();
      loadScriptListener.setData(tableData);
   }

   public void openXML()
   {
      if (outputArea2.isVisible())
      {
         jFrame.getContentPane().add(outputXMLFile);
         outputXMLFile.setVisible(true);
         outputArea2.setVisible(false);
         loadInfoArea.setVisible(false);
         openXML.setText("Hide XML File");

      }
      else
      {
         jFrame.getContentPane().remove(outputXMLFile);
         outputArea2.setVisible(true);
         loadInfoArea.setVisible(true);
         openXML.setText("Show XML File");
      }
   }

   public void deleteAllTables()
   {
      model.setNumRows(0);
      model2.setNumRows(0);
      tableData.clear();
      tableData2.clear();
      frameLabel.setText("No reference frame");
      loadInfo.setText(loadInstructionText);
      loadInfo.append(loadInfoText);
      loadCounter = 0;
      deletedScripts.clear();
      deletedScriptsPos.clear();
      deletedScriptsTime.clear();
   }

   public void deleteAll()
   {
      DefaultTableModel whichModel = model;
      ArrayList<ScriptObject> whichTableData = tableData;
      if ((checkWhichBox(table1check, table2check) != 3) && (checkWhichBox(table1check, table2check) != 0))
      {
         if (checkWhichBox(table1check, table2check) == 1)
         {
            // Do nothing
         }

         if (checkWhichBox(table1check, table2check) == 2)
         {
            whichModel = model2;
            whichTableData = tableData2;
         }

         whichModel.setNumRows(0);
         whichTableData.clear();
         frameLabel.setText("No reference frame");
         loadInfo.setText(loadInstructionText);
         loadInfo.append(loadInfoText);
         loadCounter = 0;
      }
      else if (checkWhichBox(table1check, table2check) == 0)
      {
         // Do nothing
      }
      else
      {
         model.setNumRows(0);
         model2.setNumRows(0);
         tableData.clear();
         tableData2.clear();
         frameLabel.setText("No reference frame");
         loadInfo.setText(loadInstructionText);
         loadInfo.append(loadInfoText);
         loadCounter = 0;
      }
   }

   public void undoDeleteSelectedRows()
   {
      if (deletedScripts.size() == 0)
         return;
      int lastPos = deletedScriptsPos.size() - 1;
      int lastScript = deletedScripts.size() - 1;
      int lastTime = deletedScriptsTime.size() - 1;
      int[] rowIndex = { deletedScriptsPos.get(lastPos) + 1 };

      tableData.add(deletedScriptsPos.get(lastPos), deletedScripts.get(lastScript));

      for (int i = 0; i < rowIndex.length; i++)
      {
         setTimeStamp(rowIndex[i], deletedScriptsTime.get(lastTime), true);
      }

      loadScriptListener.setData(tableData);
      refreshTableData();
      table.getSelectionModel().setSelectionInterval(deletedScriptsPos.get(lastPos), deletedScriptsPos.get(lastPos));
      deletedScripts.remove(lastScript);
      deletedScriptsPos.remove(lastPos);
      deletedScriptsTime.remove(lastTime);
   }

   public void setTimeStamp(int rowIndex, long timeDelta, boolean plus)
   {
      if (plus)
         timeDelta *= -1;

      for (int i = rowIndex; i < tableData.size(); i++)
      {
         long time = tableData.get(i).getTimeStamp();
         tableData.get(i).setTimeStamp(time - timeDelta);
      }
   }

}
