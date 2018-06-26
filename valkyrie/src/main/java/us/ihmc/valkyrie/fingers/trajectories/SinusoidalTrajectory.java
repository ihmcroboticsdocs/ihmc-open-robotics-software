package us.ihmc.valkyrie.fingers.trajectories;

public class SinusoidalTrajectory implements TrajectoryInterface
{
   private double max = Double.MAX_VALUE;
   private double min = Double.MIN_VALUE;

   private double initialQ;
   private double finalQ;

   private double trajectoryTime;
   private double delayTime;

   public SinusoidalTrajectory(double currentQ)
   {
      initialQ = 0.0;
      initialize(currentQ);
      finalQ = 0.0;

      trajectoryTime = 0.0;
      delayTime = 0.0;
   }

   public SinusoidalTrajectory(double currentQ, double maxLimit, double minLimit)
   {
      this(currentQ);
      max = maxLimit;
      min = minLimit;
   }

   @Override
   public void initialize(double... initialConditions)
   {
      initialQ = initialConditions[0];
   }

   @Override
   public void setGoal(double trajectoryTime, double delayTime, double... goalConditions)
   {
      this.trajectoryTime = trajectoryTime;
      this.delayTime = delayTime;
      this.finalQ = goalConditions[0];
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public double getDelayTime()
   {
      return delayTime;
   }

   @Override
   public double[] getGoalConditions()
   {
      double[] goal = new double[1];
      goal[0] = this.finalQ;
      return goal;
   }

   @Override
   public double getQ(double time)
   {
      double q = 0.0;

      if (time <= delayTime)
         q = initialQ;
      else if (delayTime < time && time < trajectoryTime)
         q = initialQ + 0.5 * (finalQ - initialQ) * (1 - Math.cos(Math.PI * (time - delayTime) / (trajectoryTime - delayTime)));
      else
         q = finalQ;

      if (q > max)
         return max;
      else if (q < min)
         return min;
      else
         return q;
   }

   @Override
   public double getQd(double time)
   {
      double qd = 0.0;

      if (time <= delayTime)
         qd = 0.0;
      else if (delayTime < time && time < trajectoryTime)
         qd = 0.5 * (finalQ - initialQ) * Math.PI / (trajectoryTime - delayTime) * Math.sin(Math.PI * (time - delayTime) / (trajectoryTime - delayTime));
      else
         qd = 0.0;
      return qd;
   }

}