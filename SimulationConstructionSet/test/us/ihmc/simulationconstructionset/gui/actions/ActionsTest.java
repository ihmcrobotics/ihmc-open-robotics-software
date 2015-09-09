package us.ihmc.simulationconstructionset.gui.actions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.awt.Dimension;
import java.awt.Point;

import org.junit.Test;

import us.ihmc.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CreateNewViewportWindowCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CropBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CutBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.NextCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.PackBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.PlayCommandExecutor;
import us.ihmc.simulationconstructionset.commands.PreviousCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RemoveCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SimulateCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepForwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AboutAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ActionsTest
{

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testCreateNewGraphWindowAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      CreateNewGraphWindowCommandExecutor executor = new CreateNewGraphWindowCommandExecutor()
      {
         public GraphArrayWindow createNewGraphWindow()
         {
            executorCalled[0] = true;
            return null;
         }
         
         public GraphArrayWindow createNewGraphWindow(String name)
         {
            throw new RuntimeException("Shouldn't call this one in the test!");
         }

         public GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, Point windowLocation, Dimension windowSize, boolean maximizeWindow)
         {      
            throw new RuntimeException("Shouldn't call this one in the test!");
         }

         public GraphArrayWindow getGraphArrayWindow(String windowName)
         {
            return null;
         }

      };

      CreateNewGraphWindowAction action = new CreateNewGraphWindowAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
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

         public ViewportWindow getViewportWindow(String windowName)
         {
            return null;
         }
      };

      CreateNewViewportWindowAction action = new CreateNewViewportWindowAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testCutBufferAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      CutBufferCommandExecutor executor = new CutBufferCommandExecutor()
      {
         public void cutBuffer()
         {
            executorCalled[0] = true;
         }
      };

      CutBufferAction action = new CutBufferAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

         public void closeAndDispose()
         {            
         }
      };

      HideShowViewportAction action = new HideShowViewportAction(executor);
      assertTrue(executorCalled[2]);

      action.actionPerformed(null);
      assertTrue(executorCalled[0]);

      action.actionPerformed(null);
      assertTrue(executorCalled[1]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testNextCameraKeyAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      NextCameraKeyCommandExecutor executor = new NextCameraKeyCommandExecutor()
      {
         public void nextCameraKey()
         {
            executorCalled[0] = true;
         }
      };

      NextCameraKeyAction action = new NextCameraKeyAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPackBufferAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      PackBufferCommandExecutor executor = new PackBufferCommandExecutor()
      {
         public void packBuffer()
         {
            executorCalled[0] = true;
         }
      };

      PackBufferAction action = new PackBufferAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPlayAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      PlayCommandExecutor executor = new PlayCommandExecutor()
      {
         public void play()
         {
            executorCalled[0] = true;
         }
      };

      PlayAction action = new PlayAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPreviousCameraKeyAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      PreviousCameraKeyCommandExecutor executor = new PreviousCameraKeyCommandExecutor()
      {
         public void previousCameraKey()
         {
            executorCalled[0] = true;
         }
      };

      PreviousCameraKeyAction action = new PreviousCameraKeyAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testRemoveCameraKeyAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      RemoveCameraKeyCommandExecutor executor = new RemoveCameraKeyCommandExecutor()
      {
         public void removeCameraKey()
         {
            executorCalled[0] = true;
         }
      };

      RemoveCameraKeyAction action = new RemoveCameraKeyAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetInPointAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      SetInPointCommandExecutor executor = new SetInPointCommandExecutor()
      {
         public void setInPoint()
         {
            executorCalled[0] = true;
         }
      };

      SetInPointAction action = new SetInPointAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetOutPointAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      SetOutPointCommandExecutor executor = new SetOutPointCommandExecutor()
      {
         public void setOutPoint()
         {
            executorCalled[0] = true;
         }
      };

      SetOutPointAction action = new SetOutPointAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSimulateAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      SimulateCommandExecutor executor = new SimulateCommandExecutor()
      {
         public void simulate()
         {
            executorCalled[0] = true;
         }
         
         public boolean isSimulating()
         {
            return false;
         }
      };

      SimulateAction action = new SimulateAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testStepBackwardAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      StepBackwardCommandExecutor executor = new StepBackwardCommandExecutor()
      {
         public void stepBackward()
         {
            executorCalled[0] = true;
         }
      };

      StepBackwardAction action = new StepBackwardAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testStepForwardAction()
   {
      final boolean[] executorCalled = new boolean[] {false};

      StepForwardCommandExecutor executor = new StepForwardCommandExecutor()
      {
         public void stepForward()
         {
            executorCalled[0] = true;
         }
      };

      StepForwardAction action = new StepForwardAction(executor);
      action.actionPerformed(null);

      assertTrue(executorCalled[0]);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
 public void testStopAction()
 {
    final boolean[] executorCalled = new boolean[] {false};

    StopCommandExecutor executor = new StopCommandExecutor()
    {
       public void stop()
       {
          executorCalled[0] = true;
       }
    };

    StopAction action = new StopAction(executor);
    action.actionPerformed(null);

    assertTrue(executorCalled[0]);
 }

	@DeployableTestMethod
	@Test(timeout=300000)
 public void testToggleCameraKeyModeAction()
 {
    final boolean[] executorCalled = new boolean[] {false};

    ToggleCameraKeyModeCommandExecutor executor = new ToggleCameraKeyModeCommandExecutor()
    {
       public void toggleCameraKeyMode()
       {
          executorCalled[0] = true;
       }
    };

    ToggleCameraKeyModeAction action = new ToggleCameraKeyModeAction(executor);
    action.actionPerformed(null);

    assertTrue(executorCalled[0]);
 }

	@DeployableTestMethod
	@Test(timeout=300000)
 public void testToggleKeyPointModeAction()
 {
    final boolean[] executorCalled = new boolean[] {false, false, false};
    final boolean[] isKeyPointModeToggled = new boolean[] {false};

    ToggleKeyPointModeCommandExecutor executor = new ToggleKeyPointModeCommandExecutor()
    {
       public void toggleKeyPointMode()
       {
          if (!isKeyPointModeToggled[0])
          {
             if (executorCalled[1]) throw new RuntimeException("Not working in order!");
             executorCalled[0] = true;
          }
          else
          {
             if (!executorCalled[0]) throw new RuntimeException("Not working in order!");
             executorCalled[1] = true;
          }
          
          isKeyPointModeToggled[0] = !isKeyPointModeToggled[0];
       }

      public boolean isKeyPointModeToggled()
      {
         return isKeyPointModeToggled[0];
      }

      public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener)
      {
         executorCalled[2] = true;
      }

      public void closeAndDispose()
      {         
      }
    };

    ToggleKeyPointModeAction action = new ToggleKeyPointModeAction(executor);
    assertTrue(executorCalled[2]);

    assertFalse(executor.isKeyPointModeToggled());
    
    action.actionPerformed(null);
    assertTrue(executorCalled[0]);
    assertTrue(executor.isKeyPointModeToggled());

    action.actionPerformed(null);
    assertTrue(executorCalled[1]);
    assertFalse(executor.isKeyPointModeToggled());

 }

	@DeployableTestMethod
	@Test(timeout=300000)
 public void testZoomInAction()
 {
    final boolean[] executorCalled = new boolean[] {false};

    ZoomGraphCommandExecutor executor = new ZoomGraphCommandExecutor()
    {
       public void zoomIn()
       {
          executorCalled[0] = true;
       }

      public void zoomOut()
      {
         throw new RuntimeException();
      }
    };

    ZoomInAction action = new ZoomInAction(executor);
    action.actionPerformed(null);

    assertTrue(executorCalled[0]);
 }

	@DeployableTestMethod
	@Test(timeout=300000)
 public void testZoomOutAction()
 {
    final boolean[] executorCalled = new boolean[] {false};

    ZoomGraphCommandExecutor executor = new ZoomGraphCommandExecutor()
    {
       public void zoomOut()
       {
          executorCalled[0] = true;
       }

      public void zoomIn()
      {
         throw new RuntimeException();         
      }
    };

    ZoomOutAction action = new ZoomOutAction(executor);
    action.actionPerformed(null);

    assertTrue(executorCalled[0]);
 }
 

}
