package com.yobotics.simulationconstructionset.gui.actions;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.yobotics.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import com.yobotics.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import com.yobotics.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import com.yobotics.simulationconstructionset.commands.CreateNewViewportWindowCommandExecutor;
import com.yobotics.simulationconstructionset.commands.CropBufferCommandExecutor;
import com.yobotics.simulationconstructionset.commands.GotoInPointCommandExecutor;
import com.yobotics.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import com.yobotics.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import com.yobotics.simulationconstructionset.commands.ViewportSelectorCommandListener;
import com.yobotics.simulationconstructionset.gui.ViewportWindow;
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
   public void testAddKeyPointAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      AddKeyPointCommandExecutor executor = new AddKeyPointCommandExecutor()
      {
         public void addKeyPoint()
         {
            executorCalled[0] = true;
         }
      };

      AddKeyPointAction action = new AddKeyPointAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testCreateNewGraphWindowAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      CreateNewGraphWindowCommandExecutor executor = new CreateNewGraphWindowCommandExecutor()
      {
         public void createNewGraphWindow()
         {
            executorCalled[0] = true;            
         }
         
         public void createNewGraphWindow(String name)
         {
            throw new RuntimeException("Shouldn't call this one in the test!");
         }

         public void createNewGraphWindow(String graphGroupName, int screenID, boolean maximizeWindow)
         {           
            throw new RuntimeException("Shouldn't call this one in the test!");
         }

      };

      CreateNewGraphWindowAction action = new CreateNewGraphWindowAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testCreateNewViewportWindowAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      CreateNewViewportWindowCommandExecutor executor = new CreateNewViewportWindowCommandExecutor()
      {
         public ViewportWindow createNewViewportWindow()
         {
            executorCalled[0] = true;
            return null;
         }

         public ViewportWindow createNewViewportWindow(String viewportName)
         {
            throw new RuntimeException("Shouldn't call this one in the test!");
         }

         public ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow)
         {
            throw new RuntimeException("Shouldn't call this one in the test!");
         }
      };

      CreateNewViewportWindowAction action = new CreateNewViewportWindowAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testCropBufferAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      CropBufferCommandExecutor executor = new CropBufferCommandExecutor()
      {
         public void cropBuffer()
         {
            executorCalled[0] = true;
         }
      };

      CropBufferAction action = new CropBufferAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testGotoInPointAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      GotoInPointCommandExecutor executor = new GotoInPointCommandExecutor()
      {
         public void gotoInPoint()
         {
            executorCalled[0] = true;
         }
      };

      GotoInPointAction action = new GotoInPointAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testGotoOutPointAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      GotoOutPointCommandExecutor executor = new GotoOutPointCommandExecutor()
      {
         public void gotoOutPoint()
         {
            executorCalled[0] = true;
         }
      };

      GotoOutPointAction action = new GotoOutPointAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

   @Test
   public void testHideShowViewportAction()
   {
      final boolean[] isViewportHidden = new boolean[] {false};
      final boolean[] executorCalled = new boolean[] {false, false, false};

      ViewportSelectorCommandExecutor executor = new ViewportSelectorCommandExecutor()
      {
         public void selectViewport(String name)
         {        
            throw new RuntimeException("This test shouldn't call this!");
         }

         public void hideViewport()
         {
            if (isViewportHidden[0]) throw new RuntimeException("Should not be hidden!");
            if (executorCalled[1]) throw new RuntimeException("Should hide it first!");
            executorCalled[0] = true;     
            
            isViewportHidden[0] = true;
         }

         public void showViewport()
         {           
            if (!isViewportHidden[0]) throw new RuntimeException("Should be hidden!");
            if (!executorCalled[0]) throw new RuntimeException("Should show it second!");
            executorCalled[1] = true;  
            
            isViewportHidden[0] = false;
         }

         public boolean isViewportHidden()
         {
            return isViewportHidden[0];
         }

         public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener commandListener)
         {
            executorCalled[2] = true;
         }
      };

      HideShowViewportAction action = new HideShowViewportAction(executor);
      assertTrue(executorCalled[2]);

      action.actionPerformed(null);
      assertTrue(executorCalled[0]);

      action.actionPerformed(null);
      assertTrue(executorCalled[1]);
   }

//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//
//   @Test
//   public void testGenericAction()
//   {
//      final boolean[] executorCalled = new boolean[] {false};
//
//      GenericCommandExecutor executor = new GenericCommandExecutor()
//      {
//         public void generic()
//         {
//            executorCalled[0] = true;
//         }
//      };
//
//      GenericAction action = new GenericAction(executor);
//      action.actionPerformed(null);
//
//      assertTrue(executorCalled[0]);
//   }
//


}
