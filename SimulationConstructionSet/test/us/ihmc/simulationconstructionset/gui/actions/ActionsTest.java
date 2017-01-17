package us.ihmc.simulationconstructionset.gui.actions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.awt.Dimension;
import java.awt.Point;
import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.Test;

import com.google.common.base.Defaults;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;
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
import us.ihmc.simulationconstructionset.commands.SelectGUIConfigFromFileCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SimulateCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepForwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ThinBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCameraHolder;
import us.ihmc.simulationconstructionset.gui.DollyCheckBox;
import us.ihmc.simulationconstructionset.gui.TrackCheckBox;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectGraphConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectGraphGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectVarGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.configActions.SelectViewportAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AboutAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.CameraPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.DataBufferPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportDataAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportSimulationTo3DMaxAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ExportSnapshotAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ImportDataAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.LoadConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.LoadGraphGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.LoadRobotConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.MediaCaptureAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.PlaybackPropertiesAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.PrintGraphsAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.ResizeViewportAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveGraphConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SaveRobotConfigurationAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SelectEntryBoxGroupAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.SelectExtraPanelAction;
import us.ihmc.simulationconstructionset.gui.camera.AbstractCameraPropertiesHolder;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.CameraPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.DataBufferPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportDataDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSimulationTo3DMaxDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ImportDataDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadRobotConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.MediaCaptureDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PlaybackPropertiesDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ResizeViewportDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveRobotConfigurationDialogConstructor;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = { IntegrationCategory.UI})
public class ActionsTest
{
   /**
    * List of simple actions that just delegate everything to the corresponding executors/constructors
    */
   @SuppressWarnings("serial")
   private static final Map<Class<?>, Class<?>> SIMPLE_ACTIONS = new HashMap<Class<?>, Class<?>>()
   {
      {
         put(AboutDialogConstructor.class, AboutAction.class);
         put(AddCameraKeyCommandExecutor.class, AddCameraKeyAction.class);
         put(AddKeyPointCommandExecutor.class, AddKeyPointAction.class);
         put(CropBufferCommandExecutor.class, CropBufferAction.class);
         put(CutBufferCommandExecutor.class, CutBufferAction.class);
         put(GotoInPointCommandExecutor.class, GotoInPointAction.class);
         put(GotoOutPointCommandExecutor.class, GotoOutPointAction.class);
         put(NextCameraKeyCommandExecutor.class, NextCameraKeyAction.class);
         put(PackBufferCommandExecutor.class, PackBufferAction.class);
         put(PlayCommandExecutor.class, PlayAction.class);
         put(PreviousCameraKeyCommandExecutor.class, PreviousCameraKeyAction.class);
         put(RemoveCameraKeyCommandExecutor.class, RemoveCameraKeyAction.class);
         put(SetInPointCommandExecutor.class, SetInPointAction.class);
         put(SetOutPointCommandExecutor.class, SetOutPointAction.class);
         put(StepBackwardCommandExecutor.class, StepBackwardAction.class);
         put(StepForwardCommandExecutor.class, StepForwardAction.class);
         put(StopCommandExecutor.class, StopAction.class);
         put(ToggleCameraKeyModeCommandExecutor.class, ToggleCameraKeyModeAction.class);
         put(DataBufferPropertiesDialogConstructor.class, DataBufferPropertiesAction.class);
         put(ExportDataDialogConstructor.class, ExportDataAction.class);
         put(ExportSimulationTo3DMaxDialogConstructor.class, ExportSimulationTo3DMaxAction.class);
         put(ExportSnapshotDialogConstructor.class, ExportSnapshotAction.class);
         put(ImportDataDialogConstructor.class, ImportDataAction.class);
         put(LoadConfigurationDialogConstructor.class, LoadConfigurationAction.class);
         put(LoadGraphGroupDialogConstructor.class, LoadGraphGroupAction.class);
         put(LoadRobotConfigurationDialogConstructor.class, LoadRobotConfigurationAction.class);
         put(MediaCaptureDialogConstructor.class, MediaCaptureAction.class);
         put(PlaybackPropertiesDialogConstructor.class, PlaybackPropertiesAction.class);
         put(PrintGraphsDialogConstructor.class, PrintGraphsAction.class);
         put(ResizeViewportDialogConstructor.class, ResizeViewportAction.class);
         put(SaveConfigurationDialogConstructor.class, SaveConfigurationAction.class);
         put(SaveGraphConfigurationDialogConstructor.class, SaveGraphConfigurationAction.class);
         put(SaveRobotConfigurationDialogConstructor.class, SaveRobotConfigurationAction.class);
      }
   };

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testSimpleActions()
   {
      for (Map.Entry<Class<?>, Class<?>> actionToTest : SIMPLE_ACTIONS.entrySet())
      {
         testActionCallingAllInterfaceMethods(actionToTest.getKey(), actionToTest.getValue());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testCreateNewGraphWindowAction()
   {
      createMethodTesterForInterface(CreateNewGraphWindowCommandExecutor.class)
              .testWithMock(new TestWithMock<CreateNewGraphWindowCommandExecutor>()
              {
                 @Override
                 public void test(CreateNewGraphWindowCommandExecutor mock)
                 {
                    new CreateNewGraphWindowAction(mock).actionPerformed(null);
                 }
              })
              .assertMethodCalled("createNewGraphWindow")
              .assertMethodNotCalled("createNewGraphWindow", String.class)
              .assertMethodNotCalled("createNewGraphWindow", String.class, int.class, Point.class, Dimension.class, boolean.class)
              .assertMethodNotCalled("getGraphArrayWindow", String.class);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testCreateNewViewportWindowAction()
   {
      createMethodTesterForInterface(CreateNewViewportWindowCommandExecutor.class)
              .testWithMock(new TestWithMock<CreateNewViewportWindowCommandExecutor>()
              {
                 @Override
                 public void test(CreateNewViewportWindowCommandExecutor mock)
                 {
                    new CreateNewViewportWindowAction(mock).actionPerformed(null);
                 }
              })
              .assertMethodCalled("createNewViewportWindow")
              .assertMethodNotCalled("createNewViewportWindow", String.class, int.class, boolean.class)
              .assertMethodNotCalled("createNewViewportWindow", String.class)
              .assertMethodNotCalled("getViewportWindow", String.class);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testHideShowViewportAction()
   {
      final ViewportSelectorCommandExecutor dummyExecutor = new ViewportSelectorCommandExecutor()
      {
         boolean visible = true;

         @Override
         public void selectViewport(String name) {}

         @Override
         public void hideViewport()
         {
            visible = false;
         }

         @Override
         public void showViewport()
         {
            visible = true;
         }

         @Override
         public boolean isViewportHidden()
         {
            return !visible;
         }

         @Override
         public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener commandListener) {}

         @Override
         public void closeAndDispose() {}
      };

      createMethodTesterForInterface(ViewportSelectorCommandExecutor.class, dummyExecutor)
              .testWithMock(new TestWithMock<ViewportSelectorCommandExecutor>()
              {
                 @Override
                 public void test(ViewportSelectorCommandExecutor mock)
                 {
                    HideShowViewportAction action = new HideShowViewportAction(mock);
                    assertFalse(dummyExecutor.isViewportHidden());

                    action.actionPerformed(null);
                    action.updateViewportStatus();
                    assertTrue(dummyExecutor.isViewportHidden());
                    assertTrue("Show Viewport".equals(action.getValue(HideShowViewportAction.NAME)));

                    action.actionPerformed(null);
                    action.updateViewportStatus();
                    assertTrue("Hide Viewport".equals(action.getValue(HideShowViewportAction.NAME)));
                    assertFalse(dummyExecutor.isViewportHidden());

                    action.closeAndDispose();
                 }
              })
              .assertMethodsCalledInOrder(
                      new MethodInvocation("hideViewport"),
                      new MethodInvocation("showViewport"),
                      new MethodInvocation("closeAndDispose")
              );
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testSimulateAction()
   {
      createMethodTesterForInterface(SimulateCommandExecutor.class)
              .testWithMock(new TestWithMock<SimulateCommandExecutor>()
              {
                 @Override
                 public void test(SimulateCommandExecutor mock)
                 {
                    new SimulateAction(mock).actionPerformed(null);
                 }
              })
              .assertMethodCalled("simulate");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testToggleKeyPointModeAction()
   {
      final ToggleKeyPointModeAction[] actionHolder = new ToggleKeyPointModeAction[1];

      final ToggleKeyPointModeCommandExecutor dummyExecutor = new ToggleKeyPointModeCommandExecutor()
      {
         boolean toggled = false;

         @Override
         public boolean isKeyPointModeToggled()
         {
            return toggled;
         }

         @Override
         public void toggleKeyPointMode()
         {
            toggled = !toggled;
         }

         @Override
         public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener) {}

         @Override
         public void closeAndDispose() {}
      };

      createMethodTesterForInterface(ToggleKeyPointModeCommandExecutor.class, dummyExecutor)
              .testWithMock(new TestWithMock<ToggleKeyPointModeCommandExecutor>()
              {
                 @Override
                 public void test(ToggleKeyPointModeCommandExecutor executor)
                 {
                    ToggleKeyPointModeAction action = new ToggleKeyPointModeAction(executor);
                    assertFalse(dummyExecutor.isKeyPointModeToggled());
                    action.actionPerformed(null);
                    action.updateKeyPointModeStatus();
                    assertTrue(dummyExecutor.isKeyPointModeToggled());

                    action.actionPerformed(null);
                    action.updateKeyPointModeStatus();
                    assertFalse(dummyExecutor.isKeyPointModeToggled());

                    action.closeAndDispose();
                    actionHolder[0] = action;
                 }
              })
              .assertMethodsCalledInOrder(
                      new MethodInvocation("registerToggleKeyPointModeCommandListener", actionHolder[0]),
                      new MethodInvocation("toggleKeyPointMode"),
                      new MethodInvocation("toggleKeyPointMode"),
                      new MethodInvocation("closeAndDispose")
              );
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testZoomInAction()
   {
      createMethodTesterForInterface(ZoomGraphCommandExecutor.class)
              .testWithMock(new TestWithMock<ZoomGraphCommandExecutor>()
              {
                 @Override
                 public void test(ZoomGraphCommandExecutor mock)
                 {
                    callPublicMethods(new ZoomInAction(mock));
                 }
              })
              .assertMethodCalled("zoomIn")
              .assertMethodNotCalled("zoomOut");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testZoomOutAction()
   {
      createMethodTesterForInterface(ZoomGraphCommandExecutor.class)
              .testWithMock(new TestWithMock<ZoomGraphCommandExecutor>()
              {
                 @Override
                 public void test(ZoomGraphCommandExecutor mock)
                 {
                    callPublicMethods(new ZoomOutAction(mock));
                 }
              })
              .assertMethodCalled("zoomOut")
              .assertMethodNotCalled("zoomIn");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectGraphConfigurationAction()
   {
      createMethodTesterForInterface(SelectGraphConfigurationCommandExecutor.class)
              .testWithMock(new TestWithMock<SelectGraphConfigurationCommandExecutor>()
              {
                 @Override
                 public void test(SelectGraphConfigurationCommandExecutor mock)
                 {
                    new SelectGraphConfigurationAction(mock, "Test").actionPerformed(null);
                 }
              })
              .assertMethodCalled("selectGraphConfiguration", "Test");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectGraphGroupAction()
   {
      createMethodTesterForInterface(GraphGroupSelector.class)
              .testWithMock(new TestWithMock<GraphGroupSelector>()
              {
                 @Override
                 public void test(GraphGroupSelector mock)
                 {
                    new SelectGraphGroupAction(mock, "Test").actionPerformed(null);
                 }
              })
              .assertMethodCalled("selectGraphGroup", "Test");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectVarGroupAction()
   {
      createMethodTesterForInterface(VarGroupSelector.class)
              .testWithMock(new TestWithMock<VarGroupSelector>()
              {
                 @Override
                 public void test(VarGroupSelector mock)
                 {
                    callPublicMethods(new SelectVarGroupAction(mock, "Test"));
                 }
              })
              .assertMethodCalled("selectVarGroup", "Test");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectViewportAction()
   {
      createMethodTesterForInterface(ViewportSelectorCommandExecutor.class)
              .testWithMock(new TestWithMock<ViewportSelectorCommandExecutor>()
              {
                 @Override
                 public void test(ViewportSelectorCommandExecutor mock)
                 {
                    callPublicMethods(new SelectViewportAction(mock, "Test"));
                 }
              })
              .assertMethodCalled("selectViewport", "Test");
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testCameraPropertiesAction()
   {
      final ActiveCameraHolder holder = new ActiveCameraHolder() {

         @Override
         public CameraPropertiesHolder getCameraPropertiesForActiveCamera()
         {
            return new AbstractCameraPropertiesHolder();
         }
      };
      final TrackCheckBox originalTrackCheckBox = new TrackCheckBox(holder);
      final DollyCheckBox originalDollyCheckBox = new DollyCheckBox(holder);

      createMethodTesterForInterface(CameraPropertiesDialogConstructor.class)
              .testWithMock(new TestWithMock<CameraPropertiesDialogConstructor>()
              {
                 @Override
                 public void test(CameraPropertiesDialogConstructor mock)
                 {
                    callPublicMethods(new CameraPropertiesAction(mock, originalTrackCheckBox, originalDollyCheckBox));
                 }
              })
              .assertAllInterfaceMethodsCalled();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectEntryBoxGroupAction()
   {
      createMethodTesterForInterface(EntryBoxGroupSelector.class)
              .testWithMock(new TestWithMock<EntryBoxGroupSelector>()
              {
                 @Override
                 public void test(EntryBoxGroupSelector mock)
                 {
                    callPublicMethods(new SelectEntryBoxGroupAction(mock, "Test"));
                 }
              })
              .assertMethodCalled("selectEntryBoxGroup", "Test")
              .assertAllInterfaceMethodsCalled();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectExtraPanelAction()
   {
      createMethodTesterForInterface(ExtraPanelSelector.class)
              .testWithMock(new TestWithMock<ExtraPanelSelector>()
              {
                 @Override
                 public void test(ExtraPanelSelector mock)
                 {
                    callPublicMethods(new SelectExtraPanelAction(mock, "Test"));
                 }
              })
              .assertMethodCalled("selectPanel", "Test")
              .assertAllInterfaceMethodsCalled();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testThinBufferAction()
   {
      testActionCallingAllInterfaceMethods(ThinBufferCommandExecutor.class, ThinBufferAction.class);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectCameraAction()
   {
      createMethodTesterForInterface(CameraSelector.class)
              .testWithMock(new TestWithMock<CameraSelector>()
              {
                 @Override
                 public void test(CameraSelector mock)
                 {
                    new SelectCameraAction(mock, "Test").actionPerformed(null);
                 }
              })
              .assertMethodCalled("selectCamera", "Test");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSelectGUIConfigFromFileAction()
   {
      final String testPath = "TestPath";
      createMethodTesterForInterface(SelectGUIConfigFromFileCommandExecutor.class)
              .testWithMock(new TestWithMock<SelectGUIConfigFromFileCommandExecutor>()
              {
                 @Override
                 public void test(SelectGUIConfigFromFileCommandExecutor mock)
                 {
                    new SelectGUIConfigFromFileAction(testPath, "Test", mock).actionPerformed(null);
                 }
              })
              .assertMethodCalled("selectGUIConfigFromFile", testPath);
   }

   /**
    * Test that calling all available public methods on the action invokes all available methods in the given interface.
    * @param iface interface
    * @param actionClass action to test
    * @param <Interface> interface type
    * @param <Action> action type
    */
   private static <Interface, Action> void testActionCallingAllInterfaceMethods(final Class<Interface> iface, final Class<Action> actionClass)
   {
      createMethodTesterForInterface(iface)
              .testWithMock(new TestWithMock<Interface>()
              {
                 @Override
                 public void test(Interface mock)
                 {
                    try
                    {
                       Action action = actionClass.getConstructor(iface).newInstance(mock);
                       callPublicMethodsInOrder(action, DISPOSE_LAST);
                    } catch (Exception e)
                    {
                       e.printStackTrace();
                       assertTrue(e.getMessage(), false);
                    }

                 }
              })
              .assertAllInterfaceMethodsCalled();
   }


   /**
    * Helper class holding info about a method invocation on a proxy object.
    * Used to test whether given methods were called during the test.
    */
   public static class MethodInvocation
   {
      public final String method;
      public final Object[] params;

      public MethodInvocation(String method, Object... params)
      {
         this.method = method;
         this.params = params;
      }

      /**
       * Check whether the captured parameters correspond to the given types
       * @param types parameter types
       * @return true if the parameters correspond
       */
      public boolean paramTypesMatch(Class<?>[] types)
      {
         Object[] params = this.params == null ? new Object[0] : this.params;
         types = (types == null ? new Class[0] : types);

         if (types.length != params.length)
            return false;

         for (int i = 0; i < types.length; i++)
         {
            if (params[i] == null)
            {
               if (types[i].isPrimitive())
                  return false;
               continue;
            }

            if (!types[i].isAssignableFrom(params[i].getClass()))
               return false;
         }
         return true;
      }

      /**
       * Check that the two parameter sets are equal
       * @param params1 first parameter set
       * @param params2 second parameter set
       * @return true if the two parameter sets are equal - all the objects' equals() returns true
       */
      private boolean parametersEqual(Object[] params1, Object[] params2)
      {
         if (params1 == null)
            params1 = new Object[0];
         if (params2 == null)
            params2 = new Object[0];
         return Arrays.equals(params1, params2);
      }

      @Override
      public boolean equals(Object o)
      {
         if (this == o) return true;
         if (o == null || getClass() != o.getClass()) return false;

         MethodInvocation that = (MethodInvocation) o;

         return method.equals(that.method) && parametersEqual(params, that.params);

      }

      @Override
      public int hashCode()
      {
         int result = method.hashCode();
         result = 31 * result + Arrays.hashCode(params);
         return result;
      }
   }

   /**
    * Holds a proxy instance of the given tested class or interface and logs all
    * method calls. Has a fluent API to make tests that are using it more readable.
    * @param <MonitoredClass> class whose method calls are monitored
    */
   public static class MethodCallTester<MonitoredClass> {
      private final MonitoredClass mock;
      private final Class<MonitoredClass> mockClass;
      private final List<MethodInvocation> invocations;

      MethodCallTester(MonitoredClass mock, Class<MonitoredClass> mockClass, List<MethodInvocation> invocations)
      {
         this.mock = mock;
         this.mockClass = mockClass;
         this.invocations = invocations;
      }

      /**
       * Checks whether the given method was called with the arguments given in expectedParamValues.
       * @param name method name
       * @param expectedParamValues parameter values
       * @return true if the method was called with these parameters, false otherwise
       */
      public boolean wasMethodCalled(String name, Object... expectedParamValues)
      {
         MethodInvocation expected = new MethodInvocation(name, expectedParamValues);
         for (MethodInvocation invocation : invocations)
         {
            if (invocation.equals(expected))
            {
               return true;
            }
         }
         return false;
      }

      /**
       * Checks whether the given method was called - allows distinguishing between overloads by
       * specifying the parameter types.
       * @param name method name
       * @param paramTypes method parameters
       * @return true if the method with the given parameters was called, false otherwise
       */
      public boolean wasMethodCalled(String name, Class<?>... paramTypes)
      {
         for (MethodInvocation invocation : invocations)
         {
            if (invocation.method.equals(name) && invocation.paramTypesMatch(paramTypes))
               return true;
         }
         return false;
      }

      /**
       * Checks whether the given methods were called in the given order.
       * @param methodInvocations methods that should have been called
       * @return true if the method with the given parameters was called, false otherwise
       */
      public boolean wereMethodsCalledInOrder(MethodInvocation... methodInvocations)
      {
         if (methodInvocations == null || methodInvocations.length == 0)
            return true;

         int i = 0;
         for (MethodInvocation invocation : invocations)
         {
            if (invocation.equals(methodInvocations[i]))
            {
               i++;
               if (i >= methodInvocations.length)
                  return true;
            }
         }
         return false;
      }


      public MethodCallTester<MonitoredClass> assertMethodCalled(String name, Object... expectedParamValues)
      {
         assertTrue("Method " + mockClass.getSimpleName() + "." + name + "(" + Arrays.toString(expectedParamValues) + ") was not called", wasMethodCalled(name, expectedParamValues));
         return this;
      }

      public MethodCallTester<MonitoredClass> assertMethodNotCalled(String name, Object... expectedParamValues)
      {
         assertFalse("Method " + mockClass.getSimpleName() + "." + name + "(" + Arrays.toString(expectedParamValues) + ") was called but no call was expected", wasMethodCalled(name, expectedParamValues));
         return this;
      }

      public MethodCallTester<MonitoredClass> assertMethodCalled(String name, Class<?>... paramTypes)
      {
         assertTrue("Method " + mockClass.getSimpleName() + "." + name + "(" + Arrays.toString(paramTypes) + ") was not called", wasMethodCalled(name, paramTypes));
         return this;
      }

      public MethodCallTester<MonitoredClass> assertMethodNotCalled(String name, Class<?>... paramTypes)
      {
         assertFalse("Method " + mockClass.getSimpleName() + "." + name + "(" + Arrays.toString(paramTypes) + ") was called but no call was expected", wasMethodCalled(name, paramTypes));
         return this;
      }

      public void assertMethodsCalledInOrder(MethodInvocation... methods)
      {
         assertTrue("Methods were not called in the correct oder", wereMethodsCalledInOrder(methods));
      }

      /**
       * Check that all methods declared in the given interface were called. Does *not* include methods
       * from super interfaces.
       * @return this for fluent api
       */
      public MethodCallTester<MonitoredClass> assertAllInterfaceMethodsCalled()
      {
         List<MethodInvocation> currentInvocation = new ArrayList<>(invocations); // Debugger calling toString can mess with this
         for (Method method : mockClass.getDeclaredMethods())
         {
            if (Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
               continue;

            boolean called = false;
            for (MethodInvocation invocation : currentInvocation)
            {
               if (invocation.method.equals(method.getName()))
               {
                  called = true;
                  break;
               }
            }

            assertTrue("Method " + mockClass.getSimpleName() + "." + method.getName() + " was not called", called);
         }
         return this;
      }

      /**
       * Run custom test code with the proxy instance of the monitored class.
       * @param test test code
       * @return this for fluent api
       */
      public MethodCallTester<MonitoredClass> testWithMock(TestWithMock<MonitoredClass> test)
      {
         test.test(mock);
         return this;
      }
   }

   private static Comparator<Method> DISPOSE_LAST = new Comparator<Method>()
   {
      @Override
      public int compare(Method o1, Method o2)
      {
         String n1 = o1.getName().toLowerCase(), n2 = o2.getName().toLowerCase();
         if (n1.contains("dispose") || n1.contains("close"))
            return 1;
         if (n2.contains("dispose") || n2.contains("close"))
            return -1;
         return n1.compareTo(n2);
      }
   };

   private static List<MethodInvocation> callPublicMethods(Object object)
   {
      return callPublicMethodsInOrder(object, DISPOSE_LAST);
   }

   /**
    * Calls all public methods declared in the class corresponding to the given object.
    * Does not include methods from superclasses.
    * @param object object to call the public methods on
    * @param comparator method comparator, allows running the methods in a specific order
    * @return list of methods invoked, including the parameters used for calling them
    */
   private static List<MethodInvocation> callPublicMethodsInOrder(Object object, Comparator<Method> comparator)
   {
      try
      {
         List<MethodInvocation> invocations = new ArrayList<>();
         Method[] methods = object.getClass().getDeclaredMethods();
         if (comparator != null)
            Arrays.sort(methods, comparator);

         for (Method method : methods)
         {
            if (Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
               continue;

            Object[] params = new Object[method.getParameterTypes().length];
            for (int i = 0; i < method.getParameterTypes().length; i++)
            {
               params[i] = instantiateType(method.getParameterTypes()[i]);
            }
            method.invoke(object, params);
            invocations.add(new MethodInvocation(method.getName(), params));
         }
         return invocations;
      } catch (Exception ex)
      {
         ex.printStackTrace();
         assertTrue("Error calling public methods on object " + object + " (" + object.getClass().getSimpleName() + ")", false);
         return new ArrayList<>();
      }
   }

   /**
    * Create a default object of the given type. Prefers constructors with as few
    * arguments as possible. Creates an empty proxy for interfaces.
    * @param type type to instantiate
    * @return default instance
    * @throws Exception if the default instance could not be created
    */
   private static Object instantiateType(Class<?> type) throws Exception
   {
      if (type.isPrimitive())
         return Defaults.defaultValue(type);
      else if (type == Void.class)
         return null;
      else if (type.isArray())
         return Array.newInstance(type, 0);
      else if (type.isInterface())
         return Proxy.newProxyInstance(type.getClassLoader(), new Class[]{type}, new InvocationHandler()
         {
            @Override
            public Object invoke(Object proxy, Method method, Object[] args) throws Throwable
            {
               return null;
            }
         });

      // Take a constructor with as few params as possible
      Constructor<?> constructor = type.getDeclaredConstructors()[0];
      for (Constructor<?> c : type.getDeclaredConstructors())
      {
         if (c.getParameterTypes().length < constructor.getParameterTypes().length)
            constructor = c;
      }


      Object[] params = new Object[constructor.getParameterTypes().length];
      for (int i = 0; i < constructor.getParameterTypes().length; i++)
      {
         params[i] = instantiateType(constructor.getParameterTypes()[i]);
      }
      return constructor.newInstance(params);
   }

   /**
    * Create {@link us.ihmc.simulationconstructionset.gui.actions.ActionsTest.MethodCallTester} with a proxy
    * object for the given interface that logs all method calls.
    * @param interfaceClass interface to proxy
    * @param <T> interface type
    * @return method tester instance
    */
   private static <T> MethodCallTester<T> createMethodTesterForInterface(Class<T> interfaceClass)
   {
      return createMethodTesterForInterface(interfaceClass, null);
   }

   /**
    * Create {@link us.ihmc.simulationconstructionset.gui.actions.ActionsTest.MethodCallTester} with a proxy
    * object for the given interface that logs all method calls.
    * @param interfaceClass interface to proxy
    * @param forwardInstance interface implementation that will be called when a call is logged (can be null)
    * @param <T> interface type
    * @return method tester instance
    */
   @SuppressWarnings("unchecked")
   private static <T> MethodCallTester<T> createMethodTesterForInterface(Class<T> interfaceClass, final T forwardInstance)
   {
      final List<MethodInvocation> invocations = new ArrayList<>();

      //noinspection unchecked
      T mock = (T) Proxy.newProxyInstance(interfaceClass.getClassLoader(), new Class[]{interfaceClass}, new InvocationHandler()
      {
         @Override
         public Object invoke(Object proxy, Method method, Object[] args) throws Throwable
         {
            invocations.add(new MethodInvocation(method.getName(), args));
            if (forwardInstance != null)
               return method.invoke(forwardInstance, args);
            return null;
         }
      });

      return new MethodCallTester<>(mock, interfaceClass, invocations);
   }

   private interface TestWithMock<Interface>
   {
      void test(Interface mock);
   }
}
