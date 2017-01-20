package us.ihmc.simulationconstructionset.gui.yoVariableSearch;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.JTextField;
import javax.swing.SpinnerNumberModel;
import javax.swing.TransferHandler;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.gui.DoubleClickListener;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.HorizontalSpinnerUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;

public abstract class YoVariablePanel extends JPanel implements KeyListener, MouseListener, FocusListener, ComponentListener
{
   private static final long serialVersionUID = -5860355582880221731L;

   private static ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   protected static final int SPINNER_HEIGHT = 16;
   protected static final int MINIMUM_TOTAL_WIDTH = 220;
   private static final int SPINNER_WIDTH = 84;

   protected final SelectedVariableHolder selectedVariableHolder;
   protected final YoVariablePanelJPopupMenu varPanelJPopupMenu;

   protected int selectedVariableIndex = -1;
   private boolean hasFocus = false;

   private final ArrayList<JSpinner> yoVariableSpinners = new ArrayList<JSpinner>(0);
   private final NumberFormat numberFormat;
   private DoubleClickListener doubleClickListener;

   private static final Color EMPTY_COLOR = Color.red;
   private static final Color REGULAR_COLOR = Color.black;
   private static final Color SELECTED_COLOR = Color.red;
   private static final Color SENT_COLOR = Color.blue;

   private static boolean showNameSpace = false;

   private final YoVariableSearchPanel searchPanel;

   public static void attachVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }

   public static boolean areNameSpacesShown()
   {
      return showNameSpace;
   }

   public static void removeVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.remove(listener);
   }

   public static void addNameSpaceToVarNames()
   {
      showNameSpace = !showNameSpace;
   }

   public YoVariablePanel(String name, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu, YoVariableSearchPanel searchPanel)
   {
      this.selectedVariableHolder = holder;
      this.setName(name);
      this.varPanelJPopupMenu = varPanelJPopupMenu;
      varPanelJPopupMenu.addFocusListener(this);

      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(4);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);

      this.searchPanel = searchPanel;

      init();
   }

   public YoVariablePanel(YoVariableList list, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      this(list.getName(), holder, varPanelJPopupMenu, null);
   }

   public YoVariablePanel(String name, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      this(name, holder, varPanelJPopupMenu, null);
   }

   protected abstract int getNumberOfYoVariables();

   protected abstract YoVariable<?> getYoVariable(int index);

   protected abstract List<YoVariable<?>> getAllYoVariablesCopy();

   public abstract YoVariable<?> getYoVariable(String string);

   public abstract void addChangeListener(ChangeListener changeListener);

   private void init()
   {
      this.setLayout(null);
      this.addMouseListener(this);
      this.addMouseMotionListener(new ToolTipMouseMotionListener());
      this.addKeyListener(this);
      this.setRequestFocusEnabled(true);
      this.setTransferHandler(new YoVariablePanelTransferHandler());
      this.setMinimumSize(new Dimension(MINIMUM_TOTAL_WIDTH, SPINNER_HEIGHT));
      this.addComponentListener(this);
      selectedVariableHolder.addChangeListener(new UpdateUIChangeListener());
   
      this.addFocusListener(this);
   }

   protected void clearAndSetUpTextFields()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            yoVariableSpinners.clear();
   
            for (int i = 0; i < getNumberOfYoVariables(); i++)
            {
               addTextFieldForVariable(getYoVariable(i));
            }
         }
      });
   }

   protected void addTextFieldForVariable(final YoVariable<?> yoVariable)
   {
      JSpinner spinner = createSpinnerForVariable(yoVariable);
      synchronized (yoVariableSpinners)
      {
         yoVariableSpinners.add(spinner);
      }
      add(spinner);
      
      setPanelSize(getNumberOfYoVariables());
   }

   private JSpinner createSpinnerForVariable(final YoVariable<?> yoVariable)
   {
      SpinnerNumberModel spinnerNumberModel = new SpinnerNumberModel();
      spinnerNumberModel.setStepSize(yoVariable.getStepSize());
      spinnerNumberModel.setValue(yoVariable.getValueAsDouble());
      
      final JSpinner spinner = new JSpinner(spinnerNumberModel);
      spinner.setName(yoVariable.getFullNameWithNameSpace());
      
      String tooltip = "<html>You can scroll to modify the current value<br />Press shift to be more precise OR Control to change by bigger increments</html>";
      spinner.setToolTipText(tooltip);
      spinner.addMouseWheelListener(new MouseWheelListener()
      {
         private static final double SCROLL_FACTOR = 1;
         private static final double SHIFT_FACTOR = 0.1;
         private static final double CTRL_FACTOR = 10;

         @Override
         public void mouseWheelMoved(MouseWheelEvent mouseWheelEvent)
         {
            double modifierFactor = (mouseWheelEvent.isShiftDown()) ? SHIFT_FACTOR : (mouseWheelEvent.isControlDown()) ? CTRL_FACTOR : 1;

            double currentValue = Double.parseDouble(spinner.getValue().toString());
            double delta = yoVariable.getStepSize() * mouseWheelEvent.getWheelRotation() * SCROLL_FACTOR * modifierFactor;
            spinner.setValue(currentValue + delta);
         }
      });
      spinner.setUI(new HorizontalSpinnerUI());
      
      spinner.setBorder(null);
      spinner.setBackground(getBackground());

      int width = SPINNER_WIDTH;
      int height = SPINNER_HEIGHT;
      int x = getViewableWidth() - SPINNER_WIDTH;
      int y = yoVariableSpinners.size() * height;
      spinner.setBounds(x, y, width, height);

      spinner.addChangeListener(new SetYoVariableBasedOnTextFieldActionListener(spinner, yoVariable));
      spinner.addFocusListener(this);

      return spinner;
   }
   
   public void refreshPanelWidth()
   {
      for (int i = 0; i < yoVariableSpinners.size(); i++)
      {
         yoVariableSpinners.get(i).setLocation(getViewableWidth() - SPINNER_WIDTH, i * SPINNER_HEIGHT);
      }
   }

   public boolean isEmpty()
   {
      return (this.getNumberOfYoVariables() == 0);
   }

   public void setPanelSize(int nVars)
   {
      setPreferredSize(new Dimension(getViewableWidth() < MINIMUM_TOTAL_WIDTH ? MINIMUM_TOTAL_WIDTH : getViewableWidth(), SPINNER_HEIGHT * nVars));
   }
   
   public int getViewableWidth()
   {
      return (int) getVisibleRect().getWidth();
   }

   @Override
   public void paintComponent(Graphics graphics)
   {
      super.paintComponent(graphics);

      int xStringStart = 2;
      int yStringStart = 10;
      if (this.isEmpty())
      {
         graphics.setColor(EMPTY_COLOR);
         String text = new String("Empty");
         graphics.drawString(text, xStringStart, yStringStart);
      }
      else
      {
         List<YoVariable<?>> allVariables = getAllYoVariablesCopy();
         graphics.setColor(REGULAR_COLOR);

         YoVariable<?> selectedVariable = selectedVariableHolder.getSelectedVariable();
         Rectangle rectangle = this.getVisibleRect();
         int minIndexOfVisibleVariables = getMinIndexOfVisibleVariables(rectangle);
         int maxIndexOfVisibleVariables = getMaxIndexOfVisibleVariables(rectangle);

         int longestLengthAllowed = this.getWidth() - (SPINNER_WIDTH + yStringStart);


         for (int i = minIndexOfVisibleVariables; i < maxIndexOfVisibleVariables; i++)
         {
            YoVariable<?> v = allVariables.get(i);
            JTextField jTextField = null;

            synchronized (yoVariableSpinners)
            {
               if ((i >= 0) && (i < yoVariableSpinners.size()))
               {
                  jTextField = ((JSpinner.DefaultEditor) yoVariableSpinners.get(i).getEditor()).getTextField();
               }
            }

            if (jTextField != null)
            {
               if (selectedVariable == v)
               {
                  graphics.setColor(SELECTED_COLOR);
               }
               else if (v.getYoVariableRegistry().isSent())
               {
                  graphics.setColor(SENT_COLOR);
               }
               else
               {
                  graphics.setColor(REGULAR_COLOR);
               }

               String formattedName = formatName(graphics, longestLengthAllowed, v);
               graphics.drawString(formattedName, xStringStart, i * SPINNER_HEIGHT + yStringStart);

               if (!jTextField.hasFocus())
               {
                  double value = v.getValueAsDouble();
                  String text = formatDouble(value);

                  if (!text.equals(jTextField.getText()))    // otherwise there will be an infinite update loop
                  {
                     jTextField.setText(text);
                     jTextField.setCaretPosition(0);
                  }
               }
            }
         }
      }
   }

   private String formatName(Graphics graphics, int longestLengthAllowed, YoVariable<?> v)
   {
      // fairly inefficient, could do a binary search instead

      String formattedName;
      FontMetrics fontMetrics = graphics.getFontMetrics();
      String name = "";
      if (showNameSpace)
      {
         name = v.getNameSpace().getShortName() + "." + v.getName();
      }
      else
         name = v.getName();

      if (fontMetrics.stringWidth(name) < longestLengthAllowed)
      {
         formattedName = name;
      }
      else
      {
         String ellipsis = "...";
         int lengthOfEllipsis = fontMetrics.stringWidth(ellipsis);

         int firstHalfLength = 1;
         int lengthAvailableForName = longestLengthAllowed - lengthOfEllipsis;
         int lengthAvailableForNamePart = lengthAvailableForName / 2;

         while ((firstHalfLength < name.length()) && (fontMetrics.stringWidth(name.substring(0, firstHalfLength)) < lengthAvailableForNamePart))
         {
            firstHalfLength++;
         }

         firstHalfLength--;
         String firstHalf = name.substring(0, firstHalfLength);

         int lastHalfLength = name.length();
         while ((lastHalfLength >= 0) && (fontMetrics.stringWidth(name.substring(lastHalfLength, name.length())) < lengthAvailableForNamePart))
         {
            lastHalfLength--;
         }

         lastHalfLength++;
         String lastHalf = name.substring(lastHalfLength, name.length());

         formattedName = firstHalf + ellipsis + lastHalf;
      }

      return formattedName;
   }

   private String formatDouble(double value)
   {
      String text;
      if (Double.isNaN(value) || Double.isInfinite(value))
      {
         text = Double.toString(value);
      }
      else
      {
         text = numberFormat.format(value);
      }

      return text;
   }

   public SelectedVariableHolder getVariableHolder()
   {
      return selectedVariableHolder;
   }
   
   protected List<JSpinner> getYoVariableSpinners()
   {
      return yoVariableSpinners;
   }

   @Override
   public void mousePressed(MouseEvent event)
   {
      if (!this.isFocusOwner())
      {
         if (!this.requestFocusInWindow())
         {
            System.err.println("Warning: VarPanel failed to gain focus on mousePressed.");
         }
      }

      varPanelJPopupMenu.setVisible(false);

      if (!event.isMetaDown() &&!event.isAltDown())
      {
         int indexOfSelectedVariable = getIndexOfClickedVariable(event.getY());
         if ((indexOfSelectedVariable >= 0) && (indexOfSelectedVariable < yoVariableSpinners.size()))
         {
            yoVariableSpinners.get(indexOfSelectedVariable).requestFocusInWindow();
         }

         YoVariable<?> selectedVariable = getClickedYoVariable(event.getY());

         if (selectedVariable != null)
         {
            selectedVariableHolder.setSelectedVariable(selectedVariable);
            JComponent c = (JComponent) event.getSource();
            TransferHandler handler = c.getTransferHandler();
            handler.exportAsDrag(c, event, TransferHandler.COPY);
            YoGraph.setSourceOfDrag(this);
            repaint();
         }
      }
      else if (event.isMetaDown() &&!event.isAltDown())
      {
         YoVariable<?> selectedVariable = getClickedYoVariable(event.getY());

         if (selectedVariable != null)
         {
            selectedVariableHolder.setSelectedVariable(selectedVariable);
            varPanelJPopupMenu.setLocation(event.getXOnScreen(), event.getYOnScreen());
            varPanelJPopupMenu.setVisible(true);
         }
      }
   }

   private YoVariable<?> getClickedYoVariable(int mouseY)
   {
      selectedVariableIndex = getIndexOfClickedVariable(mouseY);

      if (selectedVariableIndex >= 0)
      {
         return getYoVariable(selectedVariableIndex);
      }

      return null;
   }

   private int getIndexOfClickedVariable(int mouseY)
   {
      int indexOfClickedVariable = mouseY / SPINNER_HEIGHT;
      if ((indexOfClickedVariable >= 0) && (indexOfClickedVariable < this.getNumberOfYoVariables()))
      {
         return indexOfClickedVariable;
      }

      return -1;
   }

   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }

   @Override
   public void mouseClicked(MouseEvent evt)
   {
      int y = evt.getY();

      if (y < 0)
      {
         return;
      }

      selectedVariableIndex = y / SPINNER_HEIGHT;

      if (this.isEmpty())
      {
         selectedVariableIndex = -1;
      }
      else if (selectedVariableIndex > this.getNumberOfYoVariables())
      {
         selectedVariableIndex = -1;
      }

      if ((selectedVariableIndex >= 0) && (selectedVariableIndex < this.getNumberOfYoVariables()))
      {
         YoVariable<?> selectedVariable = getYoVariable(selectedVariableIndex);

         setSelected(selectedVariable);
      }

      // See if double click:
      if (evt.getClickCount() == 2)
      {
         YoVariable<?> selectedVariable = selectedVariableHolder.getSelectedVariable();

         if ((selectedVariable != null) && (doubleClickListener != null))
         {
            doubleClickListener.doubleClicked(selectedVariable);
         }
      }

      this.requestFocus();
      this.repaint();
   }

   public void setDoubleClickListener(DoubleClickListener listener)
   {
      this.doubleClickListener = listener;
   }

   @Override
   public void keyTyped(KeyEvent evt)
   {
   }

   @Override
   public void keyReleased(KeyEvent evt)
   {
   }

   @Override
   public void keyPressed(KeyEvent evt)
   {
      int code = evt.getKeyCode();

      switch (code)
      {
         case KeyEvent.VK_UP :
            selectedVariableIndex = Math.max(0, selectedVariableIndex - 1);

            break;

         case KeyEvent.VK_DOWN :
            selectedVariableIndex = Math.min(getNumberOfYoVariables() - 1, selectedVariableIndex + 1);

            break;
      }

      if ((selectedVariableIndex < 0) || (selectedVariableIndex > this.getNumberOfYoVariables()))
      {
         setSelected(null);
      }
      else
      {
         setSelected(getYoVariable(selectedVariableIndex));
      }

      this.repaint();
   }

   private void setSelected(YoVariable<?> var)
   {
      selectedVariableHolder.setSelectedVariable(var);
   }

   @Override
   public void focusGained(FocusEvent e)
   {
      if (e.getSource().equals(this))
      {
         if (hasFocus)
         {
            varPanelJPopupMenu.setVisible(false);
         }
      }
      else if (e.getSource() instanceof JSpinner)
      {
         JSpinner spinner = (JSpinner) e.getSource();
         JTextField textField = ((JSpinner.DefaultEditor) spinner.getEditor()).getTextField();

         Rectangle rectangle = this.getVisibleRect();
         int minIndexOfVisibleVariables = getMinIndexOfVisibleVariables(rectangle);
         int maxIndexOfVisibleVariables = getMaxIndexOfVisibleVariables(rectangle);

         int indexOfFocusedTextField = yoVariableSpinners.indexOf(spinner);
         int focusedTextFieldYPositionOnPanel = indexOfFocusedTextField * SPINNER_HEIGHT;
         if ((indexOfFocusedTextField <= minIndexOfVisibleVariables) || (indexOfFocusedTextField >= maxIndexOfVisibleVariables))
         {
            this.scrollRectToVisible(new Rectangle(0, focusedTextFieldYPositionOnPanel, spinner.getBounds().width, SPINNER_HEIGHT));
         }

         YoVariable<?> yoVariable = getYoVariable(indexOfFocusedTextField);
         if (yoVariable != null)
         {
            selectedVariableHolder.setSelectedVariable(yoVariable);
            this.repaint();
         }

         textField.setCaretPosition(textField.getText().length());
         textField.moveCaretPosition(0);
      }

      hasFocus = true;
   }

   private int getMinIndexOfVisibleVariables(Rectangle rectangle)
   {
      int size = this.getNumberOfYoVariables();
      int minIndexOfVisibleVariables = rectangle.y / SPINNER_HEIGHT;
      minIndexOfVisibleVariables = Math.max(minIndexOfVisibleVariables, 0);
      minIndexOfVisibleVariables = Math.min(minIndexOfVisibleVariables, size);

      return minIndexOfVisibleVariables;
   }

   private int getMaxIndexOfVisibleVariables(Rectangle rectangle)
   {
      int size = this.getNumberOfYoVariables();
      int maxIndexOfVisibleVariables = (rectangle.y + rectangle.height) / SPINNER_HEIGHT;
      maxIndexOfVisibleVariables = Math.max(maxIndexOfVisibleVariables, 0);
      maxIndexOfVisibleVariables = Math.min(maxIndexOfVisibleVariables, size);

      return maxIndexOfVisibleVariables;
   }

   @Override
   public void focusLost(FocusEvent e)
   {
      varPanelJPopupMenu.setVisible(false);
      hasFocus = false;
   }

   @Override
   public void componentHidden(ComponentEvent e)
   {
   }

   @Override
   public void componentMoved(ComponentEvent e)
   {
   }

   @Override
   public void componentResized(ComponentEvent e)
   {
      synchronized (yoVariableSpinners)
      {
         if (e.getSource().equals(this))
         {
            int width = this.getSize().width;
            for (JComponent textField : yoVariableSpinners)
            {
               Rectangle currentBounds = textField.getBounds();
               textField.setBounds(width - SPINNER_WIDTH, currentBounds.y, currentBounds.width, currentBounds.height);
            }
         }
      }
   }

   @Override
   public void componentShown(ComponentEvent e)
   {
   }

   private final class ToolTipMouseMotionListener implements MouseMotionListener
   {
      @Override
      public void mouseMoved(MouseEvent event)
      {
         YoVariable<?> yoVariable = getClickedYoVariable(event.getY());
         if (yoVariable != null)
         {
            String displayText = yoVariable.getFullNameWithNameSpace();
            String descriptionText = yoVariable.getDescription();
            if (!(descriptionText.equals(null) || descriptionText.equals("")))
            {
               displayText += "\nDescription: (" + descriptionText + ")";
            }

            if(searchPanel != null && searchPanel.showInitStackTrace())
            {
               displayText += "\n Initialized " + yoVariable.getStackTraceAtInitialization();
            }

            displayText = displayText.replaceAll("\n", "<br>");
            displayText = "<html>" + displayText + "</html>";
            
            setToolTipText(displayText);
         }

      }

      @Override
      public void mouseDragged(MouseEvent e)
      {
      }
   }


   private final class UpdateUIChangeListener implements ChangeListener
   {
      @Override
      public void stateChanged(ChangeEvent e)
      {
         updateUI();
      }
   }

   private final class SetYoVariableBasedOnTextFieldActionListener implements ChangeListener
   {
      private final JSpinner spinner;
      private final YoVariable<?> variable;

      private SetYoVariableBasedOnTextFieldActionListener(JSpinner spinner, YoVariable<?> variable)
      {
         this.spinner = spinner;
         this.variable = variable;
      }

      @Override
      public void stateChanged(ChangeEvent e)
      {
         try
         {
            variable.setValueFromDouble(Double.parseDouble(spinner.getValue().toString()));
         }
         catch (RuntimeException exceptionToIgnore)
         {
         }

         spinner.setValue(variable.getValueAsDouble());

         for (int i = 0; i < variableChangedListeners.size(); i++)
         {
            VariableChangedListener listener = variableChangedListeners.get(i);
            listener.variableChanged(variable);
         }

         spinner.requestFocusInWindow();
      }
   }
}
