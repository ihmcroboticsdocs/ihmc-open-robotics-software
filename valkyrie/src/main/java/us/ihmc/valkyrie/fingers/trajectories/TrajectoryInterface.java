package us.ihmc.valkyrie.fingers.trajectories;

public interface TrajectoryInterface
{
   public abstract void initialize(double... initialConditions);

   public abstract void setGoal(double trajectoryTime, double delayTime, double... goalConditions);

   public abstract double getTrajectoryTime();

   public abstract double getDelayTime();

   public abstract double[] getGoalConditions();

   public abstract double getQ(double time);

   public abstract double getQd(double time);

}