package us.ihmc.simulationconstructionset.commands;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;

public interface AllCommandsExecutor
        extends DataBufferCommandsExecutor, RunCommandsExecutor, AddCameraKeyCommandExecutor, AddKeyPointCommandExecutor, CreateNewGraphWindowCommandExecutor,
                CreateNewViewportWindowCommandExecutor, CropBufferCommandExecutor, PackBufferCommandExecutor, CutBufferCommandExecutor, ThinBufferCommandExecutor,
                NextCameraKeyCommandExecutor, PreviousCameraKeyCommandExecutor, RemoveCameraKeyCommandExecutor,
                SelectGUIConfigFromFileCommandExecutor, SetInPointCommandExecutor, SetOutPointCommandExecutor, StepBackwardCommandExecutor, StepForwardCommandExecutor,
                ToggleCameraKeyModeCommandExecutor, ToggleKeyPointModeCommandExecutor, ViewportSelectorCommandExecutor,
                ZoomGraphCommandExecutor, ExportSnapshotCommandExecutor, GUIEnablerAndDisabler
{
}
