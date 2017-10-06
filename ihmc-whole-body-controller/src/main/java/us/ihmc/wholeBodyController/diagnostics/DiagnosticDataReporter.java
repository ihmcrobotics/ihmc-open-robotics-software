package us.ihmc.wholeBodyController.diagnostics;

public interface DiagnosticDataReporter extends Runnable
{
   public abstract boolean isDoneExportingData();
}
