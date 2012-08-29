package com.yobotics.simulationconstructionset.gui.actions;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.yobotics.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import com.yobotics.simulationconstructionset.commands.SimulateCommandExecutor;
import com.yobotics.simulationconstructionset.gui.actions.dialogActions.AboutAction;
import com.yobotics.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;

public class ActionsTest
{
   @Test
   public void testAboutAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      AboutDialogConstructor executor = new AboutDialogConstructor()
      {
         public void constructAboutDialog()
         {
            executorCalled[0] = true;
         }
      };

      AboutAction aboutAction = new AboutAction(executor);
      aboutAction.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testAddCameraKeyAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      AddCameraKeyCommandExecutor executor = new AddCameraKeyCommandExecutor()
      {
         public void addCameraKey()
         {
            executorCalled[0] = true;
         }
      };

      AddCameraKeyAction action = new AddCameraKeyAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testSimulateAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      SimulateCommandExecutor executor = new SimulateCommandExecutor()
      {
         public void simulate()
         {
            executorCalled[0] = true;
         }
      };

      SimulateAction action = new SimulateAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }



}
