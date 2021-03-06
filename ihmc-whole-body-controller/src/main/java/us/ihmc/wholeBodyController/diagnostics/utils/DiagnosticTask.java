package us.ihmc.wholeBodyController.diagnostics.utils;

import java.util.ArrayDeque;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticDataReporter;

public abstract class DiagnosticTask
{
   private YoDouble timeInCurrentTask;

   public abstract void doTransitionIntoAction();
   public abstract void doAction();
   public abstract void doTransitionOutOfAction();

   public abstract boolean isDone();
   public abstract boolean abortRequested();
   public abstract String getName();
   public abstract void attachParentYoVariableRegistry(YoVariableRegistry parentRegistry);

   void setYoTimeInCurrentTask(YoDouble timeInCurrentTask)
   {
      this.timeInCurrentTask = timeInCurrentTask;
   }

   public double getTimeInCurrentTask()
   {
      return timeInCurrentTask.getDoubleValue();
   }

   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      return 0.0;
   }

   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      return 0.0;
   }

   public double getDesiredJointTauOffset(OneDoFJoint joint)
   {
      return 0.0;
   }

   /**
    *  Use this method to require the high level controller to run a dataReporter.
    *  Make sure that only one thread is running at a time.
    */
   public void getDataReporterToRun(ArrayDeque<DiagnosticDataReporter> dataReportersToPack)
   {
   }
}
