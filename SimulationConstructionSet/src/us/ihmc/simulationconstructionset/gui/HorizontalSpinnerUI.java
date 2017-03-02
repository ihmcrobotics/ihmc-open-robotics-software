package us.ihmc.simulationconstructionset.gui;

import java.awt.Component;
import java.awt.ComponentOrientation;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Insets;
import java.awt.LayoutManager;
import java.awt.event.ActionListener;
import java.awt.event.MouseListener;

import javax.swing.AbstractAction;
import javax.swing.DefaultListCellRenderer.UIResource;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JSpinner;
import javax.swing.SwingConstants;
import javax.swing.UIManager;
import javax.swing.border.Border;
import javax.swing.border.CompoundBorder;
import javax.swing.plaf.basic.BasicArrowButton;
import javax.swing.plaf.basic.BasicSpinnerUI;

public class HorizontalSpinnerUI extends BasicSpinnerUI
{
   private Component westButton;
   private Component eastButton;
   protected static final Dimension zeroSize = new Dimension(0, 0);
   static final String BASIC_SPINNERUI_LISTENER_NAME = "javax.swing.plaf.basic.BasicSpinnerUI$ArrowButtonHandler";

   @Override
   public void installUI(JComponent component)
   {
      this.spinner = (JSpinner) component;
      installDefaults();
      installListeners();
      westButton = createTheArrowButton(SwingConstants.WEST, "West");
      spinner.add(westButton);
      spinner.add(createEditor(), "Editor");
      eastButton = createTheArrowButton(SwingConstants.EAST, "East");
      spinner.add(eastButton);

      // last item in installKeyboardActions() sets action map
      installKeyboardActions();

      // Remove default Handler listener and install anew.
      initButtonListeners();
   }

   private Component createTheArrowButton(int direction, String name)
   {
      JButton button = new BasicArrowButton(direction);
      button.setName(name);
      Border buttonBorder = UIManager.getBorder("Spinner.arrowButtonBorder");
      if (buttonBorder instanceof UIResource)
      {
         // Wrap the border to avoid having the UIResource be replaced by
         // the ButtonUI. This is the opposite of using BorderUIResource.
         button.setBorder(new CompoundBorder(buttonBorder, null));
      }
      else
      {
         button.setBorder(buttonBorder);
      }

      button.setInheritsPopupMenu(true);

      return button;
   }

   @Override
   protected LayoutManager createLayout()
   {
      LayoutManager handlerLayout = new HandlerLayoutManager();

      return handlerLayout;
   }



   public void initButtonListeners()
   {
      ComponentOrientation orient = this.spinner.getComponentOrientation();
      if (orient.equals(ComponentOrientation.UNKNOWN) || orient.equals(ComponentOrientation.LEFT_TO_RIGHT))
      {
         installPreviousButtonListeners(westButton);
         installNextButtonListeners(eastButton);
      }
      else
      {
         if (orient.equals(ComponentOrientation.RIGHT_TO_LEFT))
         {
            installPreviousButtonListeners(eastButton);
            installNextButtonListeners(westButton);
         }
      }
   }

   protected boolean isBasicSpinnerUIListener(Object listener)
   {
      boolean handler = false;
      Class<?> basicSpinnerClass = javax.swing.plaf.basic.BasicSpinnerUI.class;
      if (listener != null)
      {
         try
         {
            if (listener.getClass().getEnclosingClass().equals(basicSpinnerClass) && listener.getClass().getDeclaringClass().equals(basicSpinnerClass)
                    && listener.getClass().isMemberClass() && listener.getClass().getName().equals(HorizontalSpinnerUI.BASIC_SPINNERUI_LISTENER_NAME))
            {
               handler = true;
            }
         }
         catch (NullPointerException ignore)
         {
         }
      }

      return handler;
   }

   @Override
   protected void installPreviousButtonListeners(Component c)
   {
      uninstallArrowButtonListeners(c);
      c.setName("Spinner.previousButton");
      super.installPreviousButtonListeners(c);

      if (c instanceof JButton)
      {
         ActionListener[] actionListeners = ((JButton) c).getActionListeners();
         for (int a = 0; a < actionListeners.length; a++)
         {
            if (isBasicSpinnerUIListener(actionListeners[a]))
            {
               spinner.getActionMap().put("decrement", (AbstractAction) actionListeners[a]);
            }
         }
      }
   }

   @Override
   protected void installNextButtonListeners(Component c)
   {
      uninstallArrowButtonListeners(c);
      c.setName("Spinner.nextButton");
      super.installNextButtonListeners(c);

      if (c instanceof JButton)
      {
         ActionListener[] actionListeners = ((JButton) c).getActionListeners();
         for (int a = 0; a < actionListeners.length; a++)
         {
            // Hack
            if (isBasicSpinnerUIListener(actionListeners[a]))
            {
               spinner.getActionMap().put("increment", (AbstractAction) actionListeners[a]);
            }
         }
      }
   }

   protected void uninstallArrowButtonListeners(Component component)
   {
      if (component != null)
      {
         MouseListener[] mouseListeners = component.getMouseListeners();
         for (int m = 0; m < mouseListeners.length; m++)
         {
            if (isBasicSpinnerUIListener(mouseListeners[m]))
            {
               component.removeMouseListener(mouseListeners[m]);
            }
         }

         if (component instanceof JButton)
         {
            ActionListener[] actionListeners = ((JButton) component).getActionListeners();
            for (int a = 0; a < actionListeners.length; a++)
            {
               if (isBasicSpinnerUIListener(actionListeners[a]))
               {
                  ((JButton) component).removeActionListener(actionListeners[a]);
               }
            }
         }
      }
   }

   class HandlerLayoutManager implements LayoutManager
   {
      private Component editor = null;

      @Override
      public void addLayoutComponent(String name, Component c)
      {
         if ("East".equals(name))
         {
            eastButton = c;
         }
         else if ("West".equals(name))
         {
            westButton = c;
         }
         else if ("Editor".equals(name))
         {
            editor = c;
         }
      }

      @Override
      public void removeLayoutComponent(Component c)
      {
         if (c == eastButton)
         {
            eastButton = null;
         }
         else if (c == westButton)
         {
            westButton = null;
         }
         else if (c == editor)
         {
            editor = null;
         }
      }

      private Dimension preferredSize(Component c)
      {
         return (c == null) ? zeroSize : c.getPreferredSize();
      }

      @Override
      public Dimension preferredLayoutSize(Container parent)
      {
         Dimension nextD = preferredSize(eastButton);
         Dimension previousD = preferredSize(westButton);
         Dimension editorD = preferredSize(editor);
         editorD.height = ((editorD.height + 1) / 2) * 2;
         Dimension size = new Dimension(editorD.width, editorD.height);
         size.width += 2 * (Math.max(nextD.width, previousD.width));
         Insets insets = parent.getInsets();
         size.width += insets.left + insets.right;
         size.height += insets.top + insets.bottom;

         return size;
      }

      @Override
      public Dimension minimumLayoutSize(Container parent)
      {
         return preferredLayoutSize(parent);
      }

      private void setBounds(Component c, int x, int y, int width, int height)
      {
         if (c != null)
         {
            c.setBounds(x, y, width, height);
         }
      }

      @Override
      public void layoutContainer(Container parent)
      {
         int width = parent.getWidth();
         int height = parent.getHeight();
         Insets insets = parent.getInsets();
         Dimension nextD = preferredSize(eastButton);
         Dimension previousD = preferredSize(westButton);
         int buttonWidth = Math.max(nextD.width, previousD.width);
         int editorHeight = height - (insets.top + insets.bottom);
         Insets buttonInsets = UIManager.getInsets("Spinner.arrowButtonInsets");
         if (buttonInsets == null)
         {
            buttonInsets = insets;
         }

         /*
          *  Deal with the spinner's componentOrientation property.
          */
         int editorWidth;
         if (parent.getComponentOrientation().isLeftToRight())
         {
            editorWidth = width - insets.left - (2 * buttonWidth) - buttonInsets.right;
            setBounds(westButton, insets.left, insets.top, buttonWidth, editorHeight);
            setBounds(editor, insets.left + buttonWidth, insets.top, editorWidth, editorHeight);
            setBounds(eastButton, insets.left + buttonWidth + editorWidth, insets.top, buttonWidth, editorHeight);
         }
         else
         {
            if (!parent.getComponentOrientation().isLeftToRight())
            {
               editorWidth = width - insets.right - (2 * buttonWidth) - buttonInsets.left;
               setBounds(westButton, width - buttonWidth - editorWidth - buttonWidth, insets.top, buttonWidth, editorHeight);
               setBounds(editor, width - buttonWidth - editorWidth, insets.top, editorWidth, editorHeight);
               setBounds(eastButton, width - buttonWidth, insets.top, buttonWidth, editorHeight);
            }
         }
      }
   }
}
